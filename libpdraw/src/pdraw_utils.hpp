/**
 * Parrot Drones Audio and Video Vector library
 * Utilities
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

#ifndef _PDRAW_UTILS_HPP_
#define _PDRAW_UTILS_HPP_

/* This file includes <ulog.h> to get ULOG_xxx definitions. In order for each
 * file to keep its own logging tag, we must ensure that ULOG_TAG is defined
 * BEFORE including this file */
#ifndef ULOG_TAG
#	error Please define ULOG_TAG before including pdraw_utils.hpp
#else
#	include <ulog.h>
#endif

#include <inttypes.h>
#include <string.h>

#include <pdraw/pdraw_defs.h>

#include <Eigen/Eigen>

#include <atomic>


/* Logging macros */
/* These macros must be either called from a member of a Loggable class,
 * of from a function where a local 'self' pointer to a Loggable class is
 * defined.
 * These macros use the same semantics as their ULOGx variants, except that
 * they add the Loggable name before the logged string
 */
#define _PDRAW_LOG_INT(_pri, _fmt, ...)                                        \
	ULOG_PRI(_pri, "%s: " _fmt, Loggable::_getCName(self), ##__VA_ARGS__)
#define PDRAW_LOGD(_fmt, ...) _PDRAW_LOG_INT(ULOG_DEBUG, _fmt, ##__VA_ARGS__)
#define PDRAW_LOGI(_fmt, ...) _PDRAW_LOG_INT(ULOG_INFO, _fmt, ##__VA_ARGS__)
#define PDRAW_LOGN(_fmt, ...) _PDRAW_LOG_INT(ULOG_NOTICE, _fmt, ##__VA_ARGS__)
#define PDRAW_LOGW(_fmt, ...) _PDRAW_LOG_INT(ULOG_WARN, _fmt, ##__VA_ARGS__)
#define PDRAW_LOGE(_fmt, ...) _PDRAW_LOG_INT(ULOG_ERR, _fmt, ##__VA_ARGS__)
#define PDRAW_LOG_ERRNO(_fmt, _err, ...)                                       \
	ULOGE_ERRNO(                                                           \
		(_err), "%s: " _fmt, Loggable::_getCName(self), ##__VA_ARGS__)
#define PDRAW_LOG_ERRNO_RETURN_IF(_cond, _err)                                 \
	do {                                                                   \
		if (ULOG_UNLIKELY(_cond)) {                                    \
			PDRAW_LOG_ERRNO("", (_err));                           \
			return;                                                \
		}                                                              \
	} while (0)
#define PDRAW_LOG_ERRNO_RETURN_ERR_IF(_cond, _err)                             \
	do {                                                                   \
		if (ULOG_UNLIKELY(_cond)) {                                    \
			int __pdraw_errno__err = (_err);                       \
			PDRAW_LOG_ERRNO("", (__pdraw_errno__err));             \
			return -(__pdraw_errno__err);                          \
		}                                                              \
	} while (0)
#define PDRAW_LOG_ERRNO_RETURN_VAL_IF(_cond, _err, _val)                       \
	do {                                                                   \
		if (ULOG_UNLIKELY(_cond)) {                                    \
			PDRAW_LOG_ERRNO("", (_err));                           \
			/* codecheck_ignore[RETURN_PARENTHESES] */             \
			return (_val);                                         \
		}                                                              \
	} while (0)


#define PDRAW_STATIC_ASSERT(x) typedef char __STATIC_ASSERT__[(x) ? 1 : -1]


void pdraw_gaussianDistribution(float *samples,
				unsigned int sampleCount,
				float sigma);


void pdraw_friendlyTimeFromUs(uint64_t time,
			      unsigned int *hrs,
			      unsigned int *min,
			      unsigned int *sec,
			      unsigned int *msec);


const char *
pdraw_demuxerAutodecodingModeStr(enum pdraw_demuxer_autodecoding_mode val);


enum pdraw_demuxer_autodecoding_mode
pdraw_demuxerAutodecodingModeFromStr(const char *val);


const char *pdraw_playbackTypeStr(enum pdraw_playback_type val);


enum pdraw_playback_type pdraw_playbackTypeFromStr(const char *val);


const char *pdraw_mediaTypeStr(enum pdraw_media_type val);


enum pdraw_media_type pdraw_mediaTypeFromStr(const char *val);


const char *
pdraw_muxerConnectionStateStr(enum pdraw_muxer_connection_state val);


const char *
pdraw_muxerDisconnectionReasonStr(enum pdraw_muxer_disconnection_reason val);


const char *pdraw_videoTypeStr(enum pdraw_video_type val);


enum pdraw_video_type pdraw_videoTypeFromStr(const char *val);


const char *pdraw_histogramChannelStr(enum pdraw_histogram_channel val);


enum pdraw_histogram_channel pdraw_histogramChannelFromStr(const char *val);


const char *pdraw_videoRendererSchedulingModeStr(
	enum pdraw_video_renderer_scheduling_mode val);


enum pdraw_video_renderer_scheduling_mode
pdraw_videoRendererSchedulingModeFromStr(const char *val);


const char *
pdraw_videoRendererFillModeStr(enum pdraw_video_renderer_fill_mode val);


enum pdraw_video_renderer_fill_mode
pdraw_videoRendererFillModeFromStr(const char *val);


const char *pdraw_videoRendererTransitionFlagStr(
	enum pdraw_video_renderer_transition_flag val);


enum pdraw_video_renderer_transition_flag
pdraw_videoRendererTransitionFlagFromStr(const char *val);


const char *pdraw_vipcSourceEosReasonStr(enum pdraw_vipc_source_eos_reason val);


enum pdraw_vipc_source_eos_reason
pdraw_vipcSourceEosReasonFromStr(const char *val);


int pdraw_frameMetadataToJson(const struct pdraw_video_frame *frame,
			      struct vmeta_frame *metadata,
			      struct json_object *jobj);


int pdraw_frameMetadataToJsonStr(const struct pdraw_video_frame *frame,
				 struct vmeta_frame *metadata,
				 char *output,
				 unsigned int len);

uint64_t pdraw_getTimestampFromMbufFrame(struct mbuf_coded_video_frame *frame,
					 const char *key);

uint64_t pdraw_getTimestampFromMbufFrame(struct mbuf_raw_video_frame *frame,
					 const char *key);

uint64_t pdraw_getTimestampFromMbufFrame(struct mbuf_audio_frame *frame,
					 const char *key);

struct pdraw_media_info *pdraw_mediaInfoDup(const struct pdraw_media_info *src);

void pdraw_mediaInfoFree(struct pdraw_media_info *media_info);

/* */


static inline char *xstrdup(const char *s)
{
	return s == nullptr ? nullptr : strdup(s);
}


static inline int xstrcmp(const char *s1, const char *s2)
{
	if (s1 == nullptr && s2 == nullptr)
		return 0;
	else if (s1 == nullptr)
		return -1;
	else if (s2 == nullptr)
		return 1;
	return strcmp(s1, s2);
}


static inline unsigned int pdraw_gcd(unsigned int a, unsigned int b)
{
	while (b != 0) {
		int t = a % b;
		a = b;
		b = t;
	}
	return a;
}


/* Loggable internals */

namespace Pdraw {
class Loggable {
public:
	std::string &getName()
	{
		return mName;
	}

	const char *getCName()
	{
		return mName.c_str();
	}

	/* Helper function for log macros, do not call directly */
	static const char *_getCName(Loggable *l)
	{
		if (l == nullptr)
			return "(NULL)";
		return l->getCName();
	}

protected:
	Loggable();

	void setName(std::string &name);

	void setName(const char *name);

	std::string mName;
	static std::atomic<unsigned int> mIdCounter;
	Loggable *self;
};

} /* namespace Pdraw */

#endif /* !_PDRAW_UTILS_HPP_ */
