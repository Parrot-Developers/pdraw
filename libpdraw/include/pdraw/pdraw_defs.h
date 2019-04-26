/**
 * Parrot Drones Awesome Video Viewer Library
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

#ifndef _PDRAW_DEFS_H_
#define _PDRAW_DEFS_H_

/* To be used for all public API */
#ifdef PDRAW_API_EXPORTS
#	ifdef _WIN32
#		define PDRAW_API __declspec(dllexport)
#	else /* !_WIN32 */
#		define PDRAW_API __attribute__((visibility("default")))
#	endif /* !_WIN32 */
#else /* !PDRAW_API_EXPORTS */
#	define PDRAW_API
#endif /* !PDRAW_API_EXPORTS */

#include <inttypes.h>

#include <libpomp.h>
#include <video-buffers/vbuf.h>
#include <video-metadata/vmeta.h>


/* Forward declarations */
struct egl_display;
struct json_object;
struct mux_ctx;
struct pdraw_video_renderer;
struct pdraw_video_sink;


/* Absolute maximum value of playback speed for records; if the requested
 * speed is either greater than PDRAW_PLAY_SPEED_MAX of less than
 * -PDRAW_PLAY_SPEED_MAX, the video is played as fast as possible */
#define PDRAW_PLAY_SPEED_MAX 1000.f


/* Drone model */
enum pdraw_drone_model {
	/* Unknown drone model */
	PDRAW_DRONE_MODEL_UNKNOWN = 0,

	/* Parrot Bebop */
	PDRAW_DRONE_MODEL_BEBOP,

	/* Parrot Bebop 2 */
	PDRAW_DRONE_MODEL_BEBOP2,

	/* Parrot Disco */
	PDRAW_DRONE_MODEL_DISCO,

	/* Parrot Bluegrass */
	PDRAW_DRONE_MODEL_BLUEGRASS,

	/* Parrot Anafi */
	PDRAW_DRONE_MODEL_ANAFI,

	/* Parrot Anafi Thermal */
	PDRAW_DRONE_MODEL_ANAFI_THERMAL,
};


/* Head-mounted display model */
enum pdraw_hmd_model {
	/* Unknown HMD model */
	PDRAW_HMD_MODEL_UNKNOWN = 0,

	/* Parrot Cockpit Glasses */
	PDRAW_HMD_MODEL_COCKPITGLASSES = 0,

	/* Parrot Cockpit Glasses 2 */
	PDRAW_HMD_MODEL_COCKPITGLASSES_2,
};


/* Pipeline mode */
enum pdraw_pipeline_mode {
	/* Decode all the selected media (required for rendering) */
	PDRAW_PIPELINE_MODE_DECODE_ALL = 0,

	/* Decode none of the selected media (useful to just forward
	 * the received media outside of libpdraw without rendering) */
	PDRAW_PIPELINE_MODE_DECODE_NONE,
};


/* Session type */
enum pdraw_session_type {
	/* Unknown session type */
	PDRAW_SESSION_TYPE_UNKNOWN = 0,

	/* Live stream session */
	PDRAW_SESSION_TYPE_LIVE,

	/* Replay session (either streamed or local) */
	PDRAW_SESSION_TYPE_REPLAY,
};


/* Media type */
enum pdraw_media_type {
	/* Unknown media type */
	PDRAW_MEDIA_TYPE_UNKNOWN = 0,

	/* Video media */
	PDRAW_MEDIA_TYPE_VIDEO,
};


/* Video media format */
enum pdraw_video_media_format {
	/* Unknown video media format */
	PDRAW_VIDEO_MEDIA_FORMAT_UNKNOWN = 0,

	/* YUV video media */
	PDRAW_VIDEO_MEDIA_FORMAT_YUV = (1 << 0),

	/* H.264 video media */
	PDRAW_VIDEO_MEDIA_FORMAT_H264 = (1 << 1),

	/* Opaque video media */
	PDRAW_VIDEO_MEDIA_FORMAT_OPAQUE = (1 << 2),
};


/* YUV format */
enum pdraw_yuv_format {
	/* Unknown YUV format */
	PDRAW_YUV_FORMAT_UNKNOWN = 0,

	/* "I420" YUV 4:2:0 planar (3 planes, YUV order) */
	PDRAW_YUV_FORMAT_I420,

	/* "NV12" YUV 4:2:0 semi-planar (2 planes, Y + interleaved UV) */
	PDRAW_YUV_FORMAT_NV12,
};


/* H.264 format */
enum pdraw_h264_format {
	/* Unknown H.264 format */
	PDRAW_H264_FORMAT_UNKNOWN = 0,

	/* H.264 Annex B byte stream format */
	PDRAW_H264_FORMAT_BYTE_STREAM,

	/* AVCC format (4-bytes NALU length in network order) */
	PDRAW_H264_FORMAT_AVCC,
};


/* Video type */
enum pdraw_video_type {
	/* Default camera video */
	PDRAW_VIDEO_TYPE_DEFAULT_CAMERA = 0,

	/* Front camera video */
	PDRAW_VIDEO_TYPE_FRONT_CAMERA = 0,
};


/* Histogram channel */
enum pdraw_histogram_channel {
	/* Red channel */
	PDRAW_HISTOGRAM_CHANNEL_RED = 0,

	/* Green channel */
	PDRAW_HISTOGRAM_CHANNEL_GREEN,

	/* Blue channel */
	PDRAW_HISTOGRAM_CHANNEL_BLUE,

	/* Luminance channel */
	PDRAW_HISTOGRAM_CHANNEL_LUMA,

	/* Enum values count (invalid value) */
	PDRAW_HISTOGRAM_CHANNEL_MAX,
};


/* Video renderer fill mode */
enum pdraw_video_renderer_fill_mode {
	/* Fit fill mode (the video fits in the render zone) */
	PDRAW_VIDEO_RENDERER_FILL_MODE_FIT = 0,

	/* Crop fill mode (fills the render zone, cropping the video) */
	PDRAW_VIDEO_RENDERER_FILL_MODE_CROP,

	/* Fit fill mode with blurred crop padding (the padding fills
	 * the render zone, cropping the video) */
	PDRAW_VIDEO_RENDERER_FILL_MODE_FIT_PAD_BLUR_CROP,

	/* Fit fill mode with blurred extension padding (the padding fills
	 * the render zone, extending the video sides) */
	PDRAW_VIDEO_RENDERER_FILL_MODE_FIT_PAD_BLUR_EXTEND,
};


/* Video renderer transition flag */
enum pdraw_video_renderer_transition_flag {
	/* Transition on start of stream (live and replay) */
	PDRAW_VIDEO_RENDERER_TRANSITION_FLAG_SOS = (1 << 0),

	/* Transition on end of stream (live and replay) */
	PDRAW_VIDEO_RENDERER_TRANSITION_FLAG_EOS = (1 << 1),

	/* Transition on streaming source reconfiguration (live only) */
	PDRAW_VIDEO_RENDERER_TRANSITION_FLAG_RECONFIGURE = (1 << 2),

	/* Transition on frame reception timeout (live and replay) */
	PDRAW_VIDEO_RENDERER_TRANSITION_FLAG_TIMEOUT = (1 << 3),

	/* Transition on streaming source photo trigger (live only) */
	PDRAW_VIDEO_RENDERER_TRANSITION_FLAG_PHOTO_TRIGGER = (1 << 4),
};

/* Enable all video renderer transitions */
#define PDRAW_VIDEO_RENDERER_TRANSITION_FLAG_ALL UINT32_MAX


/* Session information */
struct pdraw_session_info {
	/* Friendly name (self) */
	char friendly_name[40];

	/* Serial number (self) */
	char serial_number[32];

	/* Software version (self) */
	char software_version[20];

	/* Peer drone model */
	enum pdraw_drone_model drone_model;

	/* Session type */
	enum pdraw_session_type session_type;

	/* 1 if the session is for the drone pilot, 0 otherwise */
	int is_pilot;

	/* Playback duration in microseconds (replay only, 0 otherwise) */
	uint64_t duration;
};


/* YUV video media information */
struct pdraw_video_yuv_info {
	/* Picture width in pixels */
	unsigned int width;

	/* Picture height in pixels */
	unsigned int height;

	/* Picture left crop in pixels */
	unsigned int crop_left;

	/* Picture top crop in pixels */
	unsigned int crop_top;

	/* Picture crop width in pixels */
	unsigned int crop_width;

	/* Picture crop height in pixels */
	unsigned int crop_height;

	/* Sample aspect ratio width (no unit; full
	 * aspect ratio is sar_width / sar_height) */
	unsigned int sar_width;

	/* Sample aspect ratio height (no unit; full
	 * aspect ratio is sar_width / sar_height) */
	unsigned int sar_height;

	/* Video signal range: 0 = Y [16..235], Cb&Cr [16..240]
	 * 1 = Y&Cb&Cr [0..255] */
	int full_range;

	/* Picture horizontal field of view in degrees (0.0 means unknown) */
	float horizontal_fov;

	/* Picture vertical field of view in degrees (0.0 means unknown) */
	float vertical_fov;
};


/* H.264 video media information */
struct pdraw_video_h264_info {
	/* Picture width in pixels */
	unsigned int width;

	/* Picture height in pixels */
	unsigned int height;

	/* H.264 SPS (raw NALU without start code) */
	uint8_t sps[64];

	/* H.264 SPS size in bytes */
	size_t spslen;

	/* H.264 PPS (raw NALU without start code) */
	uint8_t pps[64];

	/* H.264 PPS size in bytes */
	size_t ppslen;
};


/* Video media information */
struct pdraw_video_info {
	/* Video media format */
	enum pdraw_video_media_format format;

	/* Video type */
	enum pdraw_video_type type;

	union {
		/* YUV video media information */
		struct pdraw_video_yuv_info yuv;

		/* H.264 video media information */
		struct pdraw_video_h264_info h264;
	};
};


/* Media information */
struct pdraw_media_info {
	/* Media type */
	enum pdraw_media_type type;

	/* Media identifier (unique within a same libpdraw instance) */
	unsigned int id;

	union {
		/* Video media information */
		struct pdraw_video_info video;
	};
};


/* YUV video frame information */
struct pdraw_video_yuv_frame {
	/* YUV format */
	enum pdraw_yuv_format format;

	/* Planes data pointers (the planes count depends on the
	 * chroma format; 0 for unused planes) */
	const uint8_t *plane[3];

	/* Planes stride in bytes (the planes count depends on the
	 * chroma format; 0 for unused planes) */
	unsigned int stride[3];

	/* Picture width in pixels */
	unsigned int width;

	/* Picture height in pixels */
	unsigned int height;

	/* Picture left crop in pixels */
	unsigned int crop_left;

	/* Picture top crop in pixels */
	unsigned int crop_top;

	/* Picture crop width in pixels */
	unsigned int crop_width;

	/* Picture crop height in pixels */
	unsigned int crop_height;

	/* Sample aspect ratio width (no unit; full
	 * aspect ratio is sar_width / sar_height) */
	unsigned int sar_width;

	/* Sample aspect ratio height (no unit; full
	 * aspect ratio is sar_width / sar_height) */
	unsigned int sar_height;

	/* Video signal range: 0 = Y [16..235], Cb&Cr [16..240]
	 * 1 = Y&Cb&Cr [0..255] */
	int full_range;
};


/* H.264 video frame information */
struct pdraw_video_h264_frame {
	/* H.264 format */
	enum pdraw_h264_format format;

	/* 1 if the frame is syntactically complete, 0 otherwise */
	int is_complete;

	/* 1 if the frame is a synchronization sample (IDR frame),
	 * 0 otherwise */
	int is_sync;

	/* 1 if the frame is a reference frame, 0 otherwise */
	int is_ref;
};


/* Video frame information */
struct pdraw_video_frame {
	/* Video media format */
	enum pdraw_video_media_format format;

	union {
		/* YUV video frame information */
		struct pdraw_video_yuv_frame yuv;

		/* H.264 video frame information */
		struct pdraw_video_h264_frame h264;
	};

	/* 1 if the frame has errors (either missing slices or error
	 * propagation from a missing slice in reference frames),
	 * 0 otherwise; errors are on H.264 video frames but the has_error
	 * value is propagated to YUV frames after decoding */
	int has_errors;

	/* 1 if the frame is to be decoded but not displayed, 0 otherwise
	 * (e.g. to mask H.264 grey IDR frames for decoder synchronization
	 * and the first intra-refresh on streams) */
	int is_silent;

	/* NTP timestamp in microseconds; this timestamp is monotonic;
	 * a 0 value can mean the timestamp is not available; for records (MP4)
	 * this is a monotonic timestamp increasing by the samples duration
	 * (i.e. it is monotonic even when seeking); when ntp_timestamp and
	 * ntp_unskewed_timestamp are not available ntp_raw_timestamp or
	 * ntp_raw_unskewed_timestamp should be used */
	uint64_t ntp_timestamp;

	/* Unskewed NTP timestamp in microseconds; same as ntp_timestamp
	 * but taking into account the clock skew between the sender and
	 * the receiver on a stream; for records (MP4) the value is identical
	 * to ntp_timestamp; a 0 value means the timestamp is not available */
	uint64_t ntp_unskewed_timestamp;

	/* Raw NTP timestamp in microseconds computed from RTP timestamp on
	 * streams; for records (MP4) the value is identical to ntp_timestamp;
	 * this timestamp is monotonic and always valid; it can be used for
	 * presentation but cannot be used for synchronization between
	 * multiple streams */
	uint64_t ntp_raw_timestamp;

	/* Unskewed raw NTP timestamp in microseconds; same as
	 * ntp_raw_timestamp but taking into account the clock skew between
	 * the sender and the receiver on a stream; for records (MP4) the value
	 * is identical to ntp_timestamp; this timestamp is always valid */
	uint64_t ntp_raw_unskewed_timestamp;

	/* Frame play timestamp in microseconds; this timestamp is always
	 * valid but it is not monotonic; on a record this timestamp
	 * corresponds to the sample decoding timestamp, i.e. the time
	 * between 0 and the record duration; on a streamed replay this
	 * timestamp corresponds to the normal play time (NPT) and has the
	 * same meaning as on a record; on a live stream this is the time
	 * since the beginning of the stream */
	uint64_t play_timestamp;

	/* Frame capture timestamp in microseconds; this timestamp is
	 * monotonic; a 0 value can mean the timestamp is not available;
	 * this timestamp corresponds to the frame acquisition time on the
	 * drone using the monotonic clock, it is primarily used for
	 * synchronization purposes with other monotonic-clock-based data
	 * such as telemetry, events or logs */
	uint64_t capture_timestamp;

	/* Local timestamp in microseconds; this timestamp is an estimation
	 * of the capture_timestamp translated on the local monotonic clock;
	 * on streams it is an estimation with a precision equal to the
	 * round-trip-delay / 2; on records (MP4) it is the sample demux time;
	 * a 0 value means the timestamp is not available; this timestamp
	 * should be used only for statistics or debugging purposes and
	 * should not be used for presentation */
	uint64_t local_timestamp;

	/* 1 if the frame metadata is valid, i.e. the metadata structure is
	 * filled, 0 otherwise */
	int has_metadata;

	/* Video frame metadata */
	struct vmeta_frame metadata;
};


/* Frame extra information */
struct pdraw_video_frame_extra {
	/* Frame play timestamp in microseconds; this timestamp is always
	 * valid but it is not monotonic; on a record this timestamp
	 * corresponds to the sample decoding timestamp, i.e. the time
	 * between 0 and the record duration; on a streamed replay this
	 * timestamp corresponds to the normal play time (NPT) and has the
	 * same meaning as on a record; on a live stream this is the time
	 * since the beginning of the stream */
	uint64_t play_timestamp;

	/* Picture histograms arrays by channel */
	float *histogram[PDRAW_HISTOGRAM_CHANNEL_MAX];

	/* Picture histograms sizes by channel; should be 256 for 8 bits
	 * formats; a 0 value means the histogram is not available */
	size_t histogram_len[PDRAW_HISTOGRAM_CHANNEL_MAX];
};


/* Rectangle */
struct pdraw_rect {
	/* Lower-left corner horizontal coordinate in pixels */
	int x;

	/* Lower-left corner vertical coordinate in pixels */
	int y;

	/* Rectangle width in pixels */
	unsigned int width;

	/* Rectangle height in pixels */
	unsigned int height;
};


/* Video renderer parameters */
struct pdraw_video_renderer_params {
	/* Renderer fill mode */
	enum pdraw_video_renderer_fill_mode fill_mode;

	/* Bitfield of enum pdraw_video_renderer_transition_flag to enable
	 * video transitions in the renderer (can be set to
	 * PDRAW_VIDEO_RENDERER_TRANSITION_FLAG_ALL in order to enable all
	 * transitions) */
	uint32_t enable_transition_flags;

	/* Enable pincushion distortion correction for head-mounted display
	 * (HMD). The HMD model and the screen parameters must be specified
	 * through the settings API. */
	int enable_hmd_distortion_correction;

	/* Inter-pupillary distance offset in millimeters, used with the HMD
	 * distortion correction. The standard IPD for a given HMD model is
	 * already applied; this is an additional offset for user settings;
	 * the default value is 0.0. */
	float hmd_ipd_offset;

	/* Horizontal offset in millimeters, used with the HMD distortion
	 * correction. The base horizontal offset should be set using the
	 * left/right device margin in the screen settings API; this is an
	 * additional offset for user settings; the default value is 0.0. */
	float hmd_x_offset;

	/* Vertical offset in millimeters, used with the HMD distortion
	 * correction. The base vertical offset should be set using the
	 * top/bottom device margin in the screen settings API; this is an
	 * additional offset for user settings; the default value is 0.0. */
	float hmd_y_offset;

	/* Scaling factor applied to the video view. This is mostly useful
	 * with the HMD distortion correction. For values < 1.0 the video is
	 * zoomed out; for values > 1.0 the video is zoomed in; the default
	 * value is 1.0. */
	float video_scale_factor;

	/* If non-null, enable the overexposure zebras on the video */
	int enable_overexposure_zebras;

	/* Overexposure zebras threshold (0.0 to 1.0); normalized pixel values
	 * above the threshold will be highlighted */
	float overexposure_zebras_threshold;

	/* If non-null, enable the picture histograms computation */
	int enable_histograms;

	/* Optional texture width in pixels to be used with the optional
	 * external texture loading callback function (ignored if no texture
	 * loading function is defined).
	 * - if video_texture_width is specified:
	 *   - if the DAR is specified (see below) the texture height is
	 *     computed according to the texture width and the DAR
	 *   - if the DAR in not specified the texture height is computed
	 *     according to the source video aspect ratio (taking the pixel
	 *     aspect ratio - SAR - into account)
	 * - if video_texture_width is not specified:
	 *   - if the DAR is specified (see below) the texture width and height
	 *     are computed according to the source video dimensions and the
	 *     DAR, so that the video texture dimensions are inscribed in the
	 *     source video dimensions
	 *   - if the DAR in not specified the texture width and height are
	 *     computed according to the source video dimensions (taking the
	 *     pixel aspect ratio - SAR - into account) */
	unsigned int video_texture_width;

	/* Optional display aspect ratio width to be used with the optional
	 * external texture loading callback function; if left null, the source
	 * video aspect ratio is used (ignored if no texture loading function
	 * is defined) */
	unsigned int video_texture_dar_width;

	/* Optional display aspect ratio height to be used with the optional
	 * external texture loading callback function; if left null, the source
	 * video aspect ratio is used (ignored if no texture loading function
	 * is defined) */
	unsigned int video_texture_dar_height;
};


/* Video sink parameters */
struct pdraw_video_sink_params {
	/* Buffer queue maximum count; optional, can be 0 which means
	 * buffers are never dropped from the queue; when not 0 and
	 * queue_drop_when_full is 1, older buffer will be automatically
	 * dropped when the queue is full to make room for new buffers;
	 * when not 0 and queue_drop_when_full is 0, newest buffers will
	 * not be queued if the queue is full */
	unsigned int queue_max_count;

	/* 1: drop oldest buffers in the queue when it is full, 0: never
	 * drop; only meaningful when queue_max_count is not 0 */
	int queue_drop_when_full;

	/* Required format for H.264 video sinks; the default value is
	 * PDRAW_H264_FORMAT_UNKNOWN which means no preference; the value
	 * is unused for non-H.264 video sinks and should be set to
	 * PDRAW_H264_FORMAT_UNKNOWN */
	enum pdraw_h264_format required_format;
};


/* Demuxer media information */
struct pdraw_demuxer_media {
	/* Media name */
	const char *name;

	/* Media id; this is the value which must be returned by the
	 * demuxer media selection callback function */
	int media_id;

	/* 1 for default media, 0 otherwise; the default media will be
	 * selected if the demuxer media selection callback function returns
	 * either 0 or -ENOSYS; note: if multiple medias are marked as
	 * default, the first one will be chosen */
	int is_default;

	/* Internal information (implementation dependant, do not use) */
	int idx;
	unsigned int stream_port;
	unsigned int control_port;
	const char *uri;
};


#endif /* !_PDRAW_DEFS_H_ */
