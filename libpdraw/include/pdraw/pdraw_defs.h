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
#	ifdef __cplusplus
/* codecheck_ignore[STORAGE_CLASS] */
#		define PDRAW_API_VAR extern "C" PDRAW_API
#	else /* !__cplusplus */
#		define PDRAW_API_VAR PDRAW_API
#	endif
#else /* !PDRAW_API_EXPORTS */
#	define PDRAW_API
/* codecheck_ignore[STORAGE_CLASS] */
#	define PDRAW_API_VAR extern
#endif /* !PDRAW_API_EXPORTS */

#include <inttypes.h>

#include <libpomp.h>
#include <media-buffers/mbuf_coded_video_frame.h>
#include <media-buffers/mbuf_raw_video_frame.h>
#include <video-defs/vdefs.h>
#include <video-metadata/vmeta.h>


/* Forward declarations */
struct egl_display;
struct json_object;
struct mux_ctx;


/* Absolute maximum value of playback speed for records; if the requested
 * speed is either greater than PDRAW_PLAY_SPEED_MAX of less than
 * -PDRAW_PLAY_SPEED_MAX, the video is played as fast as possible */
#define PDRAW_PLAY_SPEED_MAX 1000.f

/**
 * mbuf ancillary data key for pdraw_video_frame structs
 */
PDRAW_API_VAR const char *PDRAW_ANCILLARY_DATA_KEY_VIDEOFRAME;


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


/* Playback type */
enum pdraw_playback_type {
	/* Unknown playback type */
	PDRAW_PLAYBACK_TYPE_UNKNOWN = 0,

	/* Live stream */
	PDRAW_PLAYBACK_TYPE_LIVE,

	/* Replay (either streamed or local) */
	PDRAW_PLAYBACK_TYPE_REPLAY,
};


/* Media type */
enum pdraw_media_type {
	/* Unknown media type */
	PDRAW_MEDIA_TYPE_UNKNOWN = 0,

	/* Video media */
	PDRAW_MEDIA_TYPE_VIDEO,
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


/* Video renderer scheduling mode */
enum pdraw_video_renderer_scheduling_mode {
	/* Render frames as soon as possible (minimize the latency) */
	PDRAW_VIDEO_RENDERER_SCHEDULING_MODE_ASAP = 0,

	/* Adaptive jitter buffer mode (trade-off between smooth playback
	 * and minimizing the latency) */
	PDRAW_VIDEO_RENDERER_SCHEDULING_MODE_ADAPTIVE,

	/* Enum values count (invalid value) */
	PDRAW_VIDEO_RENDERER_SCHEDULING_MODE_MAX,
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

	/* Enum values count (invalid value) */
	PDRAW_VIDEO_RENDERER_FILL_MODE_MAX,
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


/* Raw video media information */
struct pdraw_raw_video_info {
	/* Raw video format */
	struct vdef_raw_format format;

	/* Format information */
	struct vdef_format_info info;
};


/* Coded video media information */
struct pdraw_coded_video_info {
	/* Coded video format */
	struct vdef_coded_format format;

	/* Format information */
	struct vdef_format_info info;

	union {
		/* H.264 information */
		struct {
			/* SPS (raw NALU without start code) */
			uint8_t sps[64];

			/* SPS size in bytes */
			size_t spslen;

			/* PPS (raw NALU without start code) */
			uint8_t pps[64];

			/* PPS size in bytes */
			size_t ppslen;
		} h264;

		/* H.265 information */
		struct {
			/* VPS (raw NALU without start code) */
			uint8_t vps[64];

			/* VPS size in bytes */
			size_t vpslen;

			/* SPS (raw NALU without start code) */
			uint8_t sps[64];

			/* SPS size in bytes */
			size_t spslen;

			/* PPS (raw NALU without start code) */
			uint8_t pps[64];

			/* PPS size in bytes */
			size_t ppslen;
		} h265;
	};
};


/* Video media information */
struct pdraw_video_info {
	/* Video media format */
	enum vdef_frame_type format;

	/* Video type */
	enum pdraw_video_type type;

	union {
		/* Raw video media information */
		struct pdraw_raw_video_info raw;

		/* Coded video media information */
		struct pdraw_coded_video_info coded;
	};
};


/* Media information */
struct pdraw_media_info {
	/* Media type */
	enum pdraw_media_type type;

	/* Media identifier (unique within a same libpdraw instance) */
	unsigned int id;

	/* Media name (unique within a same libpdraw instance) */
	const char *name;

	/* Media path (unique within a same libpdraw instance) */
	const char *path;

	/* Playback type */
	enum pdraw_playback_type playback_type;

	/* Playback duration in microseconds (replay only, 0 otherwise) */
	uint64_t duration;

	/* Session metadata */
	const struct vmeta_session *session_meta;

	union {
		/* Video media information */
		struct pdraw_video_info video;
	};
};


/* Video frame information */
struct pdraw_video_frame {
	/* Video media format */
	enum vdef_frame_type format;

	union {
		/* Raw video frame information */
		struct vdef_raw_frame raw;

		/* Coded video frame information */
		struct vdef_coded_frame coded;
	};

	/* 1 if the frame is a synchronization sample (IDR frame),
	 * 0 otherwise.
	 * Only meaningful for coded frames */
	int is_sync;

	/* 1 if the frame is a reference frame, 0 otherwise.
	 * Only meaningful for coded frames */
	int is_ref;

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
	/* Renderer frame scheduling mode */
	enum pdraw_video_renderer_scheduling_mode scheduling_mode;

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
	/* Frame queue maximum count; optional, can be 0 which means
	 * frames are never dropped from the queue; when not 0, older frames
	 * will be automatically dropped when the queue is full to make room for
	 * new frames. */
	unsigned int queue_max_count;

	/* Required format for raw or coded video sinks; the default
	 * value is VDEF_CODED_FORMAT_UNKNOWN for a coded video sink or
	 * VDEF_RAW_FORMAT_UNKNOWN for a raw video sink which means no
	 * preference */
	union {
		struct vdef_raw_format required_raw_format;
		struct vdef_coded_format required_coded_format;
	};

	/* Enable the fake frame_num generation for H.264 medias */
	int fake_frame_num;
};


/* Muxer video media parameters */
struct pdraw_muxer_video_media_params {
	/* Scaling resolution; null values mean no scaling (keep the
	 * original dimensions) */
	struct vdef_dim resolution;

	/* Transcoding format; VDEF_ENCODING_UNKNOWN means no transcoding
	 * (then scaling must be disabled with null resolution) */
	enum vdef_encoding encoding;

	/* Target bitrate in bits per second */
	unsigned int target_bitrate;

	/* GOP length in seconds (if 0.0, defaults to 1.0) */
	float gop_length_sec;
};


/* Demuxer media information */
struct pdraw_demuxer_media {
	/* Media name */
	const char *name;

	/* Media id; this is the value which must be returned by the
	 * demuxer media selection callback function */
	int media_id;

	/* 1 for default media, 0 otherwise; the default medias will be
	 * selected if the demuxer media selection callback function returns
	 * either 0 or -ENOSYS */
	int is_default;

	/* Session metadata */
	struct vmeta_session session_meta;

	/* Internal information (implementation dependant, do not use) */
	int idx;
	unsigned int stream_port;
	unsigned int control_port;
	const char *uri;
};


#endif /* !_PDRAW_DEFS_H_ */
