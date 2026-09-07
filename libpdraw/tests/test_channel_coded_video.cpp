/**
 * Parrot Drones Audio and Video Vector library
 * libpdraw unit tests — CodedVideoChannel cap getters/setters and queue()
 * null-guard (Tier B)
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

/* Channels are created implicitly by Sink::addInputMedia and accessed via
 * Sink::getInputChannel(media). The typed channel headers are transitively
 * available through test_mocks.hpp → pdraw_element.hpp → pdraw_sink.hpp. */

#define ULOG_TAG pdraw_test_channel_coded_video
#include "test_common.h"
#include "test_fixtures.hpp"
#include "test_mocks.hpp"

#include "pdraw_channel_coded_video.hpp"

#include <errno.h>

ULOG_DECLARE_TAG(ULOG_TAG);

using namespace Pdraw;
using namespace PdrawTest;


static void testCodedVideoChannelCapsAfterAdd()
{
	TestElementListener l;
	/* media must outlive sink */
	CodedVideoMedia media(g_test_session->get());
	media.format = vdef_h264_byte_stream;
	TestSinkElement sink(g_test_session->get(), &l, 4);

	CU_ASSERT_EQUAL(sink.addInputMedia(&media), 0);

	Channel *ch = sink.getInputChannel(&media);
	CU_ASSERT_PTR_NOT_NULL_FATAL(ch);

	auto *cch = dynamic_cast<CodedVideoChannel *>(ch);
	CU_ASSERT_PTR_NOT_NULL_FATAL(cch);

	const struct vdef_coded_format *caps = nullptr;
	int count = cch->getCodedVideoMediaFormatCaps(&caps);
	CU_ASSERT_EQUAL(count, 1);
	CU_ASSERT_PTR_NOT_NULL(caps);
}


static void testCodedVideoChannelCapsNullReturnsError()
{
	TestElementListener l;
	CodedVideoMedia media(g_test_session->get());
	media.format = vdef_h264_byte_stream;
	TestSinkElement sink(g_test_session->get(), &l, 4);

	sink.addInputMedia(&media);

	auto *cch =
		dynamic_cast<CodedVideoChannel *>(sink.getInputChannel(&media));
	CU_ASSERT_PTR_NOT_NULL_FATAL(cch);

	CU_ASSERT_EQUAL(cch->getCodedVideoMediaFormatCaps(nullptr), -EINVAL);
}


/* onlySupportsByteStream() has no caller anywhere in libpdraw today (grepped
 * the whole pdraw tree), but is still part of CodedVideoChannel's public
 * surface and was completely untested (0% via gcov). Two tests below cover
 * both of its branches: the loop running to completion (true) and the
 * early "return false" the first time a non-byte-stream cap is seen. */
static void testCodedVideoChannelOnlySupportsByteStreamTrueForByteStreamCaps()
{
	TestElementListener l;
	CodedVideoMedia media(g_test_session->get());
	media.format = vdef_h264_byte_stream;
	TestSinkElement sink(g_test_session->get(), &l, 4);

	CU_ASSERT_EQUAL(sink.addInputMedia(&media), 0);

	auto *cch =
		dynamic_cast<CodedVideoChannel *>(sink.getInputChannel(&media));
	CU_ASSERT_PTR_NOT_NULL_FATAL(cch);

	/* kTestSinkCodedCaps (test_mocks.cpp) is a single vdef_h264_byte_stream
	 * entry: every cap is byte-stream, so the loop must run to completion
	 * without ever hitting the early "return false". */
	CU_ASSERT_TRUE(cch->onlySupportsByteStream());
}


static void testCodedVideoChannelOnlySupportsByteStreamFalseForPacketizedCaps()
{
	TestElementListener l;
	TestSinkElement dummyOwner(g_test_session->get(), &l, 4);
	CodedVideoChannel ch(&dummyOwner,
			     nullptr,
			     nullptr,
			     g_test_session->get()->getPompLoop());

	/* Standalone channel, same construction pattern as the flush/drain/
	 * teardown tests in test_channel.cpp: setCodedVideoMediaFormatCaps()
	 * lets this test swap in a packetized (AVCC) cap, which
	 * TestSinkElement's own fixed kTestSinkCodedCaps (byte-stream only)
	 * can't exercise. */
	static const struct vdef_coded_format kPacketizedCaps[1] = {
		vdef_h264_avcc};
	ch.setCodedVideoMediaFormatCaps(&dummyOwner, kPacketizedCaps, 1);

	CU_ASSERT_FALSE(ch.onlySupportsByteStream());
}


static void testCodedVideoChannelSetCapsWrongOwnerIsNoop()
{
	/* CodedVideoChannel::setCodedVideoMediaFormatCaps() with
	 * owner != mOwner must be a no-op — caps must remain unchanged
	 * (pdraw_channel_coded_video.cpp line 65). */
	TestElementListener l;
	CodedVideoMedia media(g_test_session->get());
	media.format = vdef_h264_byte_stream;
	TestSinkElement sink(g_test_session->get(), &l, 4);

	CU_ASSERT_EQUAL(sink.addInputMedia(&media), 0);

	auto *cch =
		dynamic_cast<CodedVideoChannel *>(sink.getInputChannel(&media));
	CU_ASSERT_PTR_NOT_NULL_FATAL(cch);

	/* Read caps set by the real owner (sink). */
	const struct vdef_coded_format *capsBefore = nullptr;
	int countBefore = cch->getCodedVideoMediaFormatCaps(&capsBefore);

	/* Call with nullptr as wrong owner — must be a no-op. */
	cch->setCodedVideoMediaFormatCaps(nullptr, nullptr, 0);

	/* Caps must be unchanged. */
	const struct vdef_coded_format *capsAfter = nullptr;
	int countAfter = cch->getCodedVideoMediaFormatCaps(&capsAfter);
	CU_ASSERT_EQUAL(countAfter, countBefore);
	CU_ASSERT_PTR_EQUAL(capsAfter, capsBefore);
}


static void testCodedVideoChannelQueueNullReturnsError()
{
	/* CodedVideoChannel::queue(nullptr) must return -EINVAL
	 * (pdraw_channel_coded_video.cpp line 91). */
	TestElementListener l;
	CodedVideoMedia media(g_test_session->get());
	media.format = vdef_h264_byte_stream;
	TestSinkElement sink(g_test_session->get(), &l, 4);

	CU_ASSERT_EQUAL(sink.addInputMedia(&media), 0);

	auto *cch =
		dynamic_cast<CodedVideoChannel *>(sink.getInputChannel(&media));
	CU_ASSERT_PTR_NOT_NULL_FATAL(cch);

	int ret = cch->queue(nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);
}


CU_TestInfo g_pdraw_test_channel_coded_video[] = {
	{FN("testCodedVideoChannelCapsAfterAdd"),
	 testCodedVideoChannelCapsAfterAdd},
	{FN("testCodedVideoChannelCapsNullReturnsError"),
	 testCodedVideoChannelCapsNullReturnsError},
	{FN("testCodedVideoChannelSetCapsWrongOwnerIsNoop"),
	 testCodedVideoChannelSetCapsWrongOwnerIsNoop},
	{FN("testCodedVideoChannelQueueNullReturnsError"),
	 testCodedVideoChannelQueueNullReturnsError},
	{FN("testCodedVideoChannelOnlySupportsByteStreamTrueForByteStreamCaps"),
	 testCodedVideoChannelOnlySupportsByteStreamTrueForByteStreamCaps},
	{FN("testCodedVideoChannelOnlySupportsByteStreamFalseForPacketizedCaps"),
	 testCodedVideoChannelOnlySupportsByteStreamFalseForPacketizedCaps},
	CU_TEST_INFO_NULL,
};
