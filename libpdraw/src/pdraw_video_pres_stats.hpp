/**
 * Parrot Drones Awesome Video Viewer Library
 * Video presentation statistics
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

#ifndef _PDRAW_VIDEO_PRES_STATS_HPP_
#define _PDRAW_VIDEO_PRES_STATS_HPP_

#include <inttypes.h>

#include <libpomp.h>


namespace Pdraw {

/**
 * Video presentation statistics
 */
class VideoPresStats {
public:
	VideoPresStats(void);

	~VideoPresStats(void) {}

	/* Write video presentation statistics to a pomp_msg */
	int writeMsg(struct pomp_msg *msg, uint32_t msgid);

	/* Read video presentation statistics from a pomp_msg */
	int readMsg(const struct pomp_msg *msg);

	/* Timestamp associated with the video statistics (us, monotonic);
	 * This must be set on the receiver side to a monotonic timestamp on
	 * the sender's clock (e.g. a frame capture timestamp) */
	uint64_t timestamp;

	/* Presentation frame counter i.e. frames that reach presentation; this
	 * value can be used with the integral time values below to compute
	 * average values over a time period */
	uint32_t presentationFrameCount;

	/* Presentation frame timestamp delta integral value; timestamp delta is
	 * the time difference between two consecutive presentation frames
	 * acquisition timestamps */
	uint64_t presentationTimestampDeltaIntegral;

	/* Presentation frame timestamp delta squared integral value */
	uint64_t presentationTimestampDeltaIntegralSq;

	/* Frame presentation timing error integral value; the timing error
	 * is the absolute difference between acquisition timestamp delta
	 * and presentation timestamp delta for two consecutive presentation
	 * frames */
	uint64_t presentationTimingErrorIntegral;

	/* Frame presentation timing error squared integral value */
	uint64_t presentationTimingErrorIntegralSq;

	/* Frame estimated latency integral value; the estimated latency is the
	 * difference between a frame presentation timestamp and acquisition
	 * timestamp using an estimation of the clock difference between the
	 * sender and the receiver */
	uint64_t presentationEstimatedLatencyIntegral;

	/* Frame estimated latency squared integral value */
	uint64_t presentationEstimatedLatencyIntegralSq;

	/* Player-side frame latency integral value; the player latency is the
	 * difference between a frame presentation timestamp and the output
	 * timestamp of the frame from the reeciver */
	uint64_t playerLatencyIntegral;

	/* Player-side frame latency squared integral value */
	uint64_t playerLatencyIntegralSq;

	/* Estimated latency precision integral value; this is the precision of
	 * the estimation of the clock difference between the sender and the
	 * receiver */
	uint64_t estimatedLatencyPrecisionIntegral;
};

} /* namespace Pdraw */

#endif /* !_PDRAW_VIDEO_PRES_STATS_HPP_ */
