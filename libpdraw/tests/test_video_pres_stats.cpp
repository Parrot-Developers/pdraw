/**
 * Parrot Drones Audio and Video Vector library
 * libpdraw unit tests — VideoPresStats (Tier A)
 *
 * Copyright (c) 2026 Parrot Drones SAS
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

#include "pdraw_video_pres_stats.hpp"
#include "test_common.h"

#include <libpomp.hpp>
#include <stdint.h>

using Pdraw::VideoPresStats;


static void testWriteReadRoundtripDefault()
{
	VideoPresStats a;
	VideoPresStats b;
	pomp::Message msg;
	int ret;

	/* All fields are zero-initialised; round-trip should preserve them. */
	ret = a.writeMsg(msg, 1);
	CU_ASSERT_EQUAL(ret, 0);

	ret = b.readMsg(msg);
	CU_ASSERT_EQUAL(ret, 0);

	CU_ASSERT_EQUAL(b.timestamp, 0ULL);
	CU_ASSERT_EQUAL(b.presentationFrameCount, 0u);
	CU_ASSERT_EQUAL(b.playerLatencyIntegral, 0ULL);
}


static void testWriteReadRoundtripPopulated()
{
	VideoPresStats a;
	a.timestamp = 123456789ULL;
	a.presentationFrameCount = 300;
	a.presentationTimestampDeltaIntegral = 1000000ULL;
	a.presentationTimestampDeltaIntegralSq = 2000000ULL;
	a.presentationTimingErrorIntegral = 3000ULL;
	a.presentationTimingErrorIntegralSq = 4000ULL;
	a.presentationEstimatedLatencyIntegral = 5000ULL;
	a.presentationEstimatedLatencyIntegralSq = 6000ULL;
	a.playerLatencyIntegral = 7000ULL;
	a.playerLatencyIntegralSq = 8000ULL;
	a.estimatedLatencyPrecisionIntegral = 9000ULL;

	pomp::Message msg;
	int ret = a.writeMsg(msg, 42);
	CU_ASSERT_EQUAL(ret, 0);

	VideoPresStats b;
	ret = b.readMsg(msg);
	CU_ASSERT_EQUAL(ret, 0);

	CU_ASSERT_EQUAL(b.timestamp, a.timestamp);
	CU_ASSERT_EQUAL(b.presentationFrameCount, a.presentationFrameCount);
	CU_ASSERT_EQUAL(b.presentationTimestampDeltaIntegral,
			a.presentationTimestampDeltaIntegral);
	CU_ASSERT_EQUAL(b.presentationTimestampDeltaIntegralSq,
			a.presentationTimestampDeltaIntegralSq);
	CU_ASSERT_EQUAL(b.presentationTimingErrorIntegral,
			a.presentationTimingErrorIntegral);
	CU_ASSERT_EQUAL(b.presentationTimingErrorIntegralSq,
			a.presentationTimingErrorIntegralSq);
	CU_ASSERT_EQUAL(b.presentationEstimatedLatencyIntegral,
			a.presentationEstimatedLatencyIntegral);
	CU_ASSERT_EQUAL(b.presentationEstimatedLatencyIntegralSq,
			a.presentationEstimatedLatencyIntegralSq);
	CU_ASSERT_EQUAL(b.playerLatencyIntegral, a.playerLatencyIntegral);
	CU_ASSERT_EQUAL(b.playerLatencyIntegralSq, a.playerLatencyIntegralSq);
	CU_ASSERT_EQUAL(b.estimatedLatencyPrecisionIntegral,
			a.estimatedLatencyPrecisionIntegral);
}


static void testReadMsgTruncatedReturnsError()
{
	/* An empty default-constructed pomp::Message has a null buffer;
	 * pomp_buffer_read returns -EINVAL on a null buffer. */
	pomp::Message empty;
	VideoPresStats s;
	int ret = s.readMsg(empty);
	CU_ASSERT_EQUAL(ret, -EINVAL);
}


static void testWriteReadRoundtripBoundaryValues()
{
	VideoPresStats a;
	a.timestamp = UINT64_MAX;
	a.presentationFrameCount = UINT32_MAX;
	a.presentationTimestampDeltaIntegral = UINT64_MAX;
	a.presentationTimestampDeltaIntegralSq = UINT64_MAX;
	a.presentationTimingErrorIntegral = UINT64_MAX;
	a.presentationTimingErrorIntegralSq = UINT64_MAX;
	a.presentationEstimatedLatencyIntegral = UINT64_MAX;
	a.presentationEstimatedLatencyIntegralSq = UINT64_MAX;
	a.playerLatencyIntegral = UINT64_MAX;
	a.playerLatencyIntegralSq = UINT64_MAX;
	a.estimatedLatencyPrecisionIntegral = UINT64_MAX;

	pomp::Message msg;
	int ret = a.writeMsg(msg, 99);
	CU_ASSERT_EQUAL(ret, 0);

	VideoPresStats b;
	ret = b.readMsg(msg);
	CU_ASSERT_EQUAL(ret, 0);

	CU_ASSERT_EQUAL(b.timestamp, UINT64_MAX);
	CU_ASSERT_EQUAL(b.presentationFrameCount, UINT32_MAX);
	CU_ASSERT_EQUAL(b.presentationTimestampDeltaIntegral, UINT64_MAX);
	CU_ASSERT_EQUAL(b.presentationTimestampDeltaIntegralSq, UINT64_MAX);
	CU_ASSERT_EQUAL(b.presentationTimingErrorIntegral, UINT64_MAX);
	CU_ASSERT_EQUAL(b.presentationTimingErrorIntegralSq, UINT64_MAX);
	CU_ASSERT_EQUAL(b.presentationEstimatedLatencyIntegral, UINT64_MAX);
	CU_ASSERT_EQUAL(b.presentationEstimatedLatencyIntegralSq, UINT64_MAX);
	CU_ASSERT_EQUAL(b.playerLatencyIntegral, UINT64_MAX);
	CU_ASSERT_EQUAL(b.playerLatencyIntegralSq, UINT64_MAX);
	CU_ASSERT_EQUAL(b.estimatedLatencyPrecisionIntegral, UINT64_MAX);
}


static void testWriteMsgDifferentIds()
{
	VideoPresStats a;
	a.timestamp = 99ULL;

	pomp::Message msg1;
	pomp::Message msg2;

	CU_ASSERT_EQUAL(a.writeMsg(msg1, 10), 0);
	CU_ASSERT_EQUAL(a.writeMsg(msg2, 20), 0);

	/* Both messages are readable by the same readMsg (which ignores msgid).
	 */
	VideoPresStats b1, b2;
	CU_ASSERT_EQUAL(b1.readMsg(msg1), 0);
	CU_ASSERT_EQUAL(b2.readMsg(msg2), 0);
	CU_ASSERT_EQUAL(b1.timestamp, a.timestamp);
	CU_ASSERT_EQUAL(b2.timestamp, a.timestamp);
}


CU_TestInfo g_pdraw_test_video_pres_stats[] = {
	{FN("testWriteReadRoundtripDefault"), testWriteReadRoundtripDefault},
	{FN("testWriteReadRoundtripPopulated"),
	 testWriteReadRoundtripPopulated},
	{FN("testReadMsgTruncatedReturnsError"),
	 testReadMsgTruncatedReturnsError},
	{FN("testWriteReadRoundtripBoundaryValues"),
	 testWriteReadRoundtripBoundaryValues},
	{FN("testWriteMsgDifferentIds"), testWriteMsgDifferentIds},
	CU_TEST_INFO_NULL,
};
