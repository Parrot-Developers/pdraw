/**
 * Parrot Drones Audio and Video Vector library
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
#include <sys/types.h>

#include <audio-defs/adefs.h>
#include <audio-encode/aenc_core.h>
#include <libpomp.h>
#include <media-buffers/mbuf_audio_frame.h>
#include <media-buffers/mbuf_coded_video_frame.h>
#include <media-buffers/mbuf_raw_video_frame.h>
#include <video-defs/vdefs.h>
#include <video-encode/venc_core.h>
#include <video-metadata/vmeta.h>
#include <video-scale/vscale_core.h>


/* Forward declarations */
struct json_object;
struct mux_ctx;


/* Absolute maximum value of playback speed for records; if the requested
 * speed is either greater than PDRAW_PLAY_SPEED_MAX of less than
 * -PDRAW_PLAY_SPEED_MAX, the video is played as fast as possible */
#define PDRAW_PLAY_SPEED_MAX 1000.f

/* mbuf ancillary data key for pdraw_video_frame structs */
PDRAW_API_VAR const char *PDRAW_ANCILLARY_DATA_KEY_VIDEOFRAME;

/* mbuf ancillary data key for pdraw_audio_frame structs */
PDRAW_API_VAR const char *PDRAW_ANCILLARY_DATA_KEY_AUDIOFRAME;

/* Video renderer debug flags environment variable */
PDRAW_API_VAR const char *PDRAW_VIDEO_RENDERER_DBG_FLAGS;

/* Video renderer debug flag: macroblock status overlay (streaming only) */
#define PDRAW_VIDEO_RENDERER_DBG_FLAG_MB_STATUS_OVERLAY (1 << 0)


/* Demuxer auto-decoding mode */
enum pdraw_demuxer_autodecoding_mode {
	/* Decode all the selected media (required for rendering) */
	PDRAW_DEMUXER_AUTODECODING_MODE_DECODE_ALL = 0,

	/* Decode none of the selected media (useful to just forward
	 * the received media outside of libpdraw without rendering) */
	PDRAW_DEMUXER_AUTODECODING_MODE_DECODE_NONE,
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

	/* Audio media */
	PDRAW_MEDIA_TYPE_AUDIO,
};


/* Video type */
enum pdraw_video_type {
	/* Default camera video */
	PDRAW_VIDEO_TYPE_DEFAULT_CAMERA = 0,

	/* Front camera video */
	PDRAW_VIDEO_TYPE_FRONT_CAMERA = 0,
};


/* Muxer type */
enum pdraw_muxer_type {
	/* Unknown muxer type */
	PDRAW_MUXER_TYPE_UNKNOWN = 0,

	/* Record muxer */
	PDRAW_MUXER_TYPE_RECORD,

	/* RTMP muxer */
	PDRAW_MUXER_TYPE_RTMP,
};


/* Muxer connection state */
enum pdraw_muxer_connection_state {
	/* Unknown connection state */
	PDRAW_MUXER_CONNECTION_STATE_UNKNOWN = 0,

	/* Muxer disconnected */
	PDRAW_MUXER_CONNECTION_STATE_DISCONNECTED,

	/* Muxer connecting */
	PDRAW_MUXER_CONNECTION_STATE_CONNECTING,

	/* Muxer connected */
	PDRAW_MUXER_CONNECTION_STATE_CONNECTED,
};


/* Muxer disconnection reason */
enum pdraw_muxer_disconnection_reason {
	/* Unknown disconnection reason */
	PDRAW_MUXER_DISCONNECTION_REASON_UNKNOWN = 0,

	/* Client requested disconnection */
	PDRAW_MUXER_DISCONNECTION_REASON_CLIENT_REQUEST,

	/* Server requested disconnection */
	PDRAW_MUXER_DISCONNECTION_REASON_SERVER_REQUEST,

	/* Network error */
	PDRAW_MUXER_DISCONNECTION_REASON_NETWORK_ERROR,

	/* Connection refused by the server */
	PDRAW_MUXER_DISCONNECTION_REASON_REFUSED,

	/* Server is already in use */
	PDRAW_MUXER_DISCONNECTION_REASON_ALREADY_IN_USE,

	/* Timeout */
	PDRAW_MUXER_DISCONNECTION_REASON_TIMEOUT,

	/* Internal error */
	PDRAW_MUXER_DISCONNECTION_REASON_INTERNAL_ERROR,
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

	/* Render frames as soon as possible (minimize the latency) and
	 * signal frames only once through the renderReady listener function */
	PDRAW_VIDEO_RENDERER_SCHEDULING_MODE_ASAP_SIGNAL_ONCE,

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


/* Video IPC source end of stream reason */
enum pdraw_vipc_source_eos_reason {
	/* No reason; the video IPC is stopped for a long period or the program
	 * is exiting */
	PDRAW_VIPC_SOURCE_EOS_REASON_NONE = 0,

	/* Restart of video IPC; when the video IPC is restarting or changing
	 * its configuration, implying that it is temporarily stopped and will
	 * restart as soon as possible */
	PDRAW_VIPC_SOURCE_EOS_REASON_RESTART,

	/* A configuration change has stopped the video IPC and does not mean
	 * that the stream will restart */
	PDRAW_VIPC_SOURCE_EOS_REASON_CONFIGURATION,

	/* The video IPC is not responding and the client timed out; the video
	 * IPC source will not restart */
	PDRAW_VIPC_SOURCE_EOS_REASON_TIMEOUT,
};


/* Alsa source end of stream reason */
enum pdraw_alsa_source_eos_reason {
	/* No reason; the ALSA source is stopped for a long period or the
	 * program is exiting */
	PDRAW_ALSA_SOURCE_EOS_REASON_NONE = 0,

	/* The ALSA source encountered an unrecoverable error and will not
	   restart */
	PDRAW_ALSA_SOURCE_EOS_REASON_UNRECOVERABLE_ERROR,
};


/* Muxer thumbnail types */
enum pdraw_muxer_thumbnail_type {
	/* Unknown thumbnail type */
	PDRAW_MUXER_THUMBNAIL_TYPE_UNKNOWN = 0,

	/* JPEG thumbnail */
	PDRAW_MUXER_THUMBNAIL_TYPE_JPEG,

	/* PNG thumbnail */
	PDRAW_MUXER_THUMBNAIL_TYPE_PNG,

	/* BMP thumbnail */
	PDRAW_MUXER_THUMBNAIL_TYPE_BMP,
};


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


/* Audio media information */
struct pdraw_audio_info {
	/* Audio format */
	struct adef_format format;

	union {
		/* AAC_LC information */
		struct {
			/* ASC (Audio Spcific Config) */
			uint8_t asc[64];

			/* ASC size in bytes */
			size_t asclen;
		} aac_lc;
	};
};


/* Video media information */
struct pdraw_video_info {
	/* Video media format */
	enum vdef_frame_type format;

	/* Video type */
	enum pdraw_video_type type;

	/* Session metadata */
	const struct vmeta_session *session_meta;

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

	union {
		/* Video media information */
		struct pdraw_video_info video;

		/* Audio media information */
		struct pdraw_audio_info audio;
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
	/* Frame information */
	struct vdef_frame_info info;

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


/* Audio frame information */
struct pdraw_audio_frame {
	/* Audio frame information */
	struct adef_frame audio;

	/* NTP timestamp in microseconds; this timestamp is monotonic;
	 * a 0 value can mean the timestamp is not available; for records (MP4)
	 * this is a monotonic timestamp increasing by the frame duration
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
	 * corresponds to the frame decoding timestamp, i.e. the time
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

	/* If non-null, enable the automatic normalization of the video when
	 * range values are available in metadata (e.g. for raw thermal video
	 * tracks in records) */
	int enable_auto_normalization;

	/* If non-null, enable the overexposure zebras on the video */
	int enable_overexposure_zebras;

	/* Overexposure zebras threshold (0.0 to 1.0); normalized pixel values
	 * above the threshold will be highlighted */
	float overexposure_zebras_threshold;

	/* If non-null, enable the picture histograms computation */
	int enable_histograms;

	/* If non-null, enable a simplified version of the rendering for low
	 * GPU performace systems; in this mode auto normalization and
	 * overexposure zebras are not available, and only the TIMEOUT and
	 * RECONFIGURE transitions are available; this parameter is only
	 * available when creating a renderer, dynamic change to this
	 * parameter is ignored */
	int enable_simplified_rendering;

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


/* Audio renderer parameters */
struct pdraw_audio_renderer_params {
	/* ALSA renderer device address (cannot be dynamically reconfigured) */
	const char *address;
};


/* Video IPC source parameters */
struct pdraw_vipc_source_params {
	/* Video IPC server address */
	const char *address;

	/* Video IPC friendly name for logging; optional, can be NULL; if NULL
	 * the address is used */
	const char *friendly_name;

	/* Video IPC backend name; optional, can be NULL; if NULL the default
	 * backend is used */
	const char *backend_name;

	/* Maximum client-side frame count; optional, can be 0 which means
	 * use the default frame count; this should be less than or equal to
	 * the server-side maximum frame count */
	unsigned int frame_count;

	/* Initial video resolution to be configured; if left null the default
	 * video IPC resolution is kept; if dynamic resolution configuration
	 * is not supported, the vipcSourceConfigured() listener function or
	 * configured() callback function will be called with an error status */
	struct vdef_dim resolution;

	/* Initial image crop to be configured; if left null the default
	 * video IPC crop is kept; if dynamic crop configuration is not
	 * supported, the vipcSourceConfigured() listener function or
	 * configured() callback function will be called with an error status */
	struct vdef_rectf crop;

	/* Time scale in Hz for frame timestamps; optional, can be 0 which
	 * means use an automatic timescale based on the input framerate */
	uint32_t timescale;

	/* Applied input framerate decimation; optional, can be 0 which means
	 * no decimation (same as 1) */
	unsigned int decimation;

	/* Session metadata */
	struct vmeta_session session_meta;

	/* Timeout for the video IPC connection in ms; optional, can be 0 which
	 * means using the default value (-1 means wait indefinitely) */
	int connection_timeout_ms;

	/* Timeout for the video IPC frame reception in ms; optional, can be 0
	 * which means using the default value (-1 means wait indefinitely) */
	int frame_timeout_ms;
};


/* Video source parameters */
struct pdraw_video_source_params {
	/* Frame queue maximum count; optional, can be 0 which means
	 * frames are never dropped from the queue; when not 0, older frames
	 * will be automatically dropped when the queue is full to make room for
	 * new frames. */
	unsigned int queue_max_count;

	/* Playback type */
	enum pdraw_playback_type playback_type;

	/* Playback duration in microseconds (replay only, 0 otherwise) */
	uint64_t duration;

	/* Session metadata */
	struct vmeta_session session_meta;

	/* Video media information */
	struct pdraw_video_info video;
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
};


/* Alsa source parameters */
struct pdraw_alsa_source_params {
	/* ALSA source device address */
	const char *address;

	/* Number of samples per audio frame */
	uint32_t sample_count;

	/* Audio media information */
	struct pdraw_audio_info audio;
};


/* Alsa source capabilities */
struct pdraw_alsa_source_caps {
	/* Channel count capabilities */
	struct {
		uint8_t min;
		uint8_t max;
	} channel_count;

	/* Sampling rate capabilities */
	struct {
		uint16_t min;
		uint16_t max;
	} sample_rate;
};


/* Audio source parameters */
struct pdraw_audio_source_params {
	/* Playback type */
	enum pdraw_playback_type playback_type;

	/* Playback duration in microseconds (replay only, 0 otherwise) */
	uint64_t duration;

	/* Audio media information */
	struct pdraw_audio_info audio;
};


/* Demuxer parameters */
struct pdraw_demuxer_params {
	/* Auto-decoding mode controlling whether to decode the selected video
	 * media (for full processing up to the rendering), or to disable video
	 * decoding (e.g. when no rendering is required, only a coded video
	 * sink). */
	enum pdraw_demuxer_autodecoding_mode autodecoding_mode;
};


/* Muxer parameters */
struct pdraw_muxer_params {
	/* File mode for file creation (record muxer only); can be 0 which
	 * means use a default value of 0600 */
	mode_t filemode;

	/* Minimum required free space on the storage to write the file
	 * (record muxer only); if the free space goes below this value no more
	 * samples are recorded and the onMuxerUnrecoverableError listener
	 * function or unrecoverable_error callback function is called;
	 * optional, can be 0 which means that free space is not checked */
	size_t free_space_limit;

	/* Reserved size for MP4 tables in MB; optional, can be 0 which means
	 * use a default value (MP4_MUX_DEFAULT_TABLE_SIZE_MB) */
	size_t tables_size_mb;

	struct {
		/* File used to link the tables_file, the storage UUID and the
		 * broken file for recovery */
		char *link_file;

		/* File used to store moov box for recovery */
		char *tables_file;

		/* Tables sync period in milliseconds; optional, can be 0 which
		 * means that tables will be never sync'ed */
		uint32_t sync_period_ms;

		/* Keep a reference of the storage UUID for recovery; optional,
		 * can be false which means that storage UUID won't be check in
		 * case of recovery */
		bool check_storage_uuid;
	} recovery;

	/* Tables internal sync period in milliseconds (record muxer only);
	 * optional, can be 0 which means that tables will be never sync'ed */
	uint32_t tables_sync_period_ms;
};


struct pdraw_muxer_dyn_params {
	/* Tables internal sync period in milliseconds (record muxer only);
	 * optional, can be 0 which means that tables will be never sync'ed */
	uint32_t tables_sync_period_ms;
};


/* Muxer media parameters */
struct pdraw_muxer_media_params {
	/* Custom media name; optional; can be NULL which means use
	 * a default name ('DefaultVideo' for coded video tracks, 'RawVideo'
	 * for raw video tracks, 'DefaultAudio' for audio tracks) */
	const char *track_name;

	/* Time scale in Hz for timestamps; optional, can be 0 which means
	 * use a default timescale */
	uint32_t timescale;

	/* True if this is a default media that should be enabled by a player */
	bool is_default;
};


/* Muxer statistics */
struct pdraw_muxer_stats {
	/* Muxer type */
	enum pdraw_muxer_type type;

	union {
		struct {
			/* Number of coded video frames recorded */
			uint32_t coded_video_frames;

			/* Number of raw video frames recorded */
			uint32_t raw_video_frames;

			/* Number of audio frames recorded */
			uint32_t audio_frames;
		} record;

		struct {
			/* Wether the RTMP client is connected */
			bool is_connected;

			/* Number of video frames that are currently in the
			 * queue pending to be sent */
			uint32_t pending_video_frames;

			/* Number of audio frames that are currently in the
			 * queue pending to be sent */
			uint32_t pending_audio_frames;

			/* Number of video frames that have been dropped since
			 * the muxer started */
			uint32_t dropped_video_frames;

			/* Number of audio frames that have been dropped since
			 * the muxer started */
			uint32_t dropped_audio_frames;

			/* Max bandwidth supported by the server (B/s); 0 means
			 * that this metric is not available at the moment */
			uint32_t max_peer_bw;
		} rtmp;
	};
};


/* Demuxer video media information */
struct pdraw_demuxer_video_media {
	/* Session metadata */
	struct vmeta_session session_meta;
};


/* Demuxer audio media information */
struct pdraw_demuxer_audio_media {
	/* Reserved value */
	int reserved;
};


/* Demuxer media information */
struct pdraw_demuxer_media {
	/* Media type */
	enum pdraw_media_type type;

	/* Media name */
	const char *name;

	/* Media id; this is the value which must be returned by the
	 * demuxer media selection callback function */
	int media_id;

	/* 1 for default media, 0 otherwise; the default medias will be
	 * selected if the demuxer media selection callback function returns
	 * either 0 or -ENOSYS */
	int is_default;

	union {
		/* Video media information */
		struct pdraw_demuxer_video_media video;

		/* Audio media information */
		struct pdraw_demuxer_audio_media audio;
	};

	/* Internal information (implementation dependant, do not use) */
	int idx;
	unsigned int stream_port;
	unsigned int control_port;
	const char *uri;
};


/* Chapter item */
struct pdraw_chapter {
	/* Timestamp of the chapter (us) */
	uint64_t ts_us;

	/* Name of the chapter */
	const char *name;
};


#endif /* !_PDRAW_DEFS_H_ */
