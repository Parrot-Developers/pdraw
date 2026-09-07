/**
 * Parrot Drones Audio and Video Vector library
 * libpdraw unit tests — RawVideoChannel cap getters/setters and queue()
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

#define ULOG_TAG pdraw_test_channel_raw_video
#include "test_common.h"
#include "test_fixtures.hpp"
#include "test_mocks.hpp"

#include "pdraw_channel_raw_video.hpp"

#include <errno.h>

ULOG_DECLARE_TAG(ULOG_TAG);

using namespace Pdraw;
using namespace PdrawTest;


static void testRawVideoChannelCapsAfterAdd()
{
	TestElementListener l;
	RawVideoMedia media(g_test_session->get());
	media.format = vdef_i420;
	TestSinkElementRaw sink(g_test_session->get(), &l, 4);

	CU_ASSERT_EQUAL(sink.addInputMedia(&media), 0);

	Channel *ch = sink.getInputChannel(&media);
	CU_ASSERT_PTR_NOT_NULL_FATAL(ch);

	auto *rch = dynamic_cast<RawVideoChannel *>(ch);
	CU_ASSERT_PTR_NOT_NULL_FATAL(rch);

	const struct vdef_raw_format *caps = nullptr;
	int count = rch->getRawVideoMediaFormatCaps(&caps);
	CU_ASSERT_EQUAL(count, 1);
	CU_ASSERT_PTR_NOT_NULL(caps);
}
static void testRawVideoChannelGetCapsNullReturnsError()
{
	/* RawVideoChannel::getRawVideoMediaFormatCaps(nullptr) must return
	 * -EINVAL (pdraw_channel_raw_video.cpp). */
	TestElementListener l;
	RawVideoMedia media(g_test_session->get());
	media.format = vdef_i420;
	TestSinkElementRaw sink(g_test_session->get(), &l, 4);

	CU_ASSERT_EQUAL(sink.addInputMedia(&media), 0);

	Channel *ch = sink.getInputChannel(&media);
	CU_ASSERT_PTR_NOT_NULL_FATAL(ch);
	auto *rch = dynamic_cast<RawVideoChannel *>(ch);
	CU_ASSERT_PTR_NOT_NULL_FATAL(rch);

	int ret = rch->getRawVideoMediaFormatCaps(nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);
}


static void testRawVideoChannelQueueNullReturnsError()
{
	/* RawVideoChannel::queue(nullptr) must return -EINVAL
	 * (pdraw_channel_raw_video.cpp). */
	TestElementListener l;
	RawVideoMedia media(g_test_session->get());
	media.format = vdef_i420;
	TestSinkElementRaw sink(g_test_session->get(), &l, 4);

	CU_ASSERT_EQUAL(sink.addInputMedia(&media), 0);

	Channel *ch = sink.getInputChannel(&media);
	CU_ASSERT_PTR_NOT_NULL_FATAL(ch);
	auto *rch = dynamic_cast<RawVideoChannel *>(ch);
	CU_ASSERT_PTR_NOT_NULL_FATAL(rch);

	int ret = rch->queue(nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);
}


static void testRawVideoChannelSetCapsWrongOwnerIsNoop()
{
	/* RawVideoChannel::setRawVideoMediaFormatCaps() with owner != mOwner
	 * must be a no-op — caps must remain unchanged
	 * (pdraw_channel_raw_video.cpp). */
	TestElementListener l;
	RawVideoMedia media(g_test_session->get());
	media.format = vdef_i420;
	TestSinkElementRaw sink(g_test_session->get(), &l, 4);

	CU_ASSERT_EQUAL(sink.addInputMedia(&media), 0);

	Channel *ch = sink.getInputChannel(&media);
	CU_ASSERT_PTR_NOT_NULL_FATAL(ch);
	auto *rch = dynamic_cast<RawVideoChannel *>(ch);
	CU_ASSERT_PTR_NOT_NULL_FATAL(rch);

	/* Read the caps set by the real owner (sink). */
	const struct vdef_raw_format *capsBefore = nullptr;
	int countBefore = rch->getRawVideoMediaFormatCaps(&capsBefore);

	/* Call with nullptr as wrong owner — must be a no-op. */
	rch->setRawVideoMediaFormatCaps(nullptr, nullptr, 0);

	/* Caps must be unchanged. */
	const struct vdef_raw_format *capsAfter = nullptr;
	int countAfter = rch->getRawVideoMediaFormatCaps(&capsAfter);
	CU_ASSERT_EQUAL(countAfter, countBefore);
	CU_ASSERT_PTR_EQUAL(capsAfter, capsBefore);
}


CU_TestInfo g_pdraw_test_channel_raw_video[] = {
	{FN("testRawVideoChannelCapsAfterAdd"),
	 testRawVideoChannelCapsAfterAdd},
	{FN("testRawVideoChannelGetCapsNullReturnsError"),
	 testRawVideoChannelGetCapsNullReturnsError},
	{FN("testRawVideoChannelQueueNullReturnsError"),
	 testRawVideoChannelQueueNullReturnsError},
	{FN("testRawVideoChannelSetCapsWrongOwnerIsNoop"),
	 testRawVideoChannelSetCapsWrongOwnerIsNoop},
	CU_TEST_INFO_NULL,
};
