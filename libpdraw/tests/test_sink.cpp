/**
 * Parrot Drones Audio and Video Vector library
 * libpdraw unit tests — Sink input-port management (Tier B)
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

#define ULOG_TAG pdraw_test_sink
#include "test_common.h"
#include "test_fixtures.hpp"
#include "test_mocks.hpp"

#include "pdraw_media.hpp"

ULOG_DECLARE_TAG(ULOG_TAG);

using namespace Pdraw;
using namespace PdrawTest;


static void testSinkAddInputMediaCoded()
{
	TestElementListener l;
	/* media must outlive sink: Sink::~Sink() logs media->getName() */
	CodedVideoMedia media(g_test_session->get());
	media.format =
		vdef_h264_byte_stream; /* must match kTestSinkCodedCaps */
	TestSinkElement sink(g_test_session->get(), &l, 4);

	CU_ASSERT_EQUAL(sink.addInputMedia(&media), 0);
}


static void testSinkAddInputMediaDuplicate()
{
	TestElementListener l;
	CodedVideoMedia media(g_test_session->get()); /* outlives sink */
	media.format = vdef_h264_byte_stream;
	TestSinkElement sink(g_test_session->get(), &l, 4);

	CU_ASSERT_EQUAL(sink.addInputMedia(&media), 0);
	CU_ASSERT_EQUAL(sink.addInputMedia(&media), -EEXIST);
}


static void testSinkAddBeyondMaxRejected()
{
	TestElementListener l;
	CodedVideoMedia m1(g_test_session->get()); /* outlive sink */
	CodedVideoMedia m2(g_test_session->get());
	m1.format = vdef_h264_byte_stream;
	m2.format = vdef_h264_byte_stream;
	TestSinkElement sink(g_test_session->get(), &l, 1);

	CU_ASSERT_EQUAL(sink.addInputMedia(&m1), 0);
	CU_ASSERT_EQUAL(sink.addInputMedia(&m2), -ENOBUFS);
}


static void testSinkFindInputMediaExisting()
{
	TestElementListener l;
	CodedVideoMedia media(g_test_session->get()); /* outlives sink */
	media.format = vdef_h264_byte_stream;
	TestSinkElement sink(g_test_session->get(), &l, 4);

	sink.addInputMedia(&media);
	CU_ASSERT_PTR_EQUAL(sink.findInputMedia(&media), &media);
}


static void testSinkFindInputMediaMissing()
{
	TestElementListener l;
	CodedVideoMedia media(g_test_session->get());
	TestSinkElement sink(g_test_session->get(), &l, 4);

	CU_ASSERT_PTR_NULL(sink.findInputMedia(&media));
}


static void testSinkInputCountAfterMultipleAdds()
{
	TestElementListener l;
	CodedVideoMedia m1(g_test_session->get()); /* outlive sink */
	CodedVideoMedia m2(g_test_session->get());
	m1.format = vdef_h264_byte_stream;
	m2.format = vdef_h264_byte_stream;
	TestSinkElement sink(g_test_session->get(), &l, 4);

	CU_ASSERT_EQUAL(sink.getInputMediaCount(), 0u);
	sink.addInputMedia(&m1);
	CU_ASSERT_EQUAL(sink.getInputMediaCount(), 1u);
	sink.addInputMedia(&m2);
	CU_ASSERT_EQUAL(sink.getInputMediaCount(), 2u);
}


static void testSinkAddNullMediaReturnsError()
{
	TestElementListener l;
	TestSinkElement sink(g_test_session->get(), &l, 4);
	CU_ASSERT_EQUAL(sink.addInputMedia(nullptr), -EINVAL);
}


static void testSinkRemoveInputMedia()
{
	TestElementListener l;
	CodedVideoMedia media(g_test_session->get()); /* outlives sink */
	media.format = vdef_h264_byte_stream;
	TestSinkElement sink(g_test_session->get(), &l, 4);

	CU_ASSERT_EQUAL(sink.addInputMedia(&media), 0);
	CU_ASSERT_EQUAL(sink.getInputMediaCount(), 1u);

	CU_ASSERT_EQUAL(sink.removeInputMedia(&media), 0);
	CU_ASSERT_EQUAL(sink.getInputMediaCount(), 0u);

	/* Second removal: media no longer registered. */
	CU_ASSERT_EQUAL(sink.removeInputMedia(&media), -ENOENT);
}


static void testSinkGetInputMediaByIndex()
{
	TestElementListener l;
	CodedVideoMedia m1(g_test_session->get()); /* outlive sink */
	CodedVideoMedia m2(g_test_session->get());
	m1.format = vdef_h264_byte_stream;
	m2.format = vdef_h264_byte_stream;
	TestSinkElement sink(g_test_session->get(), &l, 4);

	sink.addInputMedia(&m1);
	sink.addInputMedia(&m2);
	CU_ASSERT_PTR_EQUAL(sink.getInputMedia(0), &m1);
	CU_ASSERT_PTR_EQUAL(sink.getInputMedia(1), &m2);
	CU_ASSERT_PTR_NULL(sink.getInputMedia(2)); /* out of range */
}


static void testSinkAddInputMediaRawVideo()
{
	TestElementListener l;
	RawVideoMedia media(g_test_session->get()); /* outlives sink */
	media.format = vdef_i420;
	TestSinkElementRaw sink(g_test_session->get(), &l, 4);

	CU_ASSERT_EQUAL(sink.addInputMedia(&media), 0);
	CU_ASSERT_EQUAL(sink.getInputMediaCount(), 1u);
}


static void testSinkAddInputMediaTearingDown()
{
	TestElementListener l;
	CodedVideoMedia media(g_test_session->get()); /* outlives sink */
	media.format = vdef_h264_byte_stream;
	media.setTearingDown();
	TestSinkElement sink(g_test_session->get(), &l, 4);

	/* isTearingDown() guard fires before any other check → -EPERM */
	CU_ASSERT_EQUAL(sink.addInputMedia(&media), -EPERM);
}


static void testSinkRemoveAllInputMedias()
{
	TestElementListener l;
	CodedVideoMedia m1(g_test_session->get()); /* outlive sink */
	CodedVideoMedia m2(g_test_session->get());
	m1.format = vdef_h264_byte_stream;
	m2.format = vdef_h264_byte_stream;
	TestSinkElement sink(g_test_session->get(), &l, 4);

	sink.addInputMedia(&m1);
	sink.addInputMedia(&m2);
	CU_ASSERT_EQUAL(sink.getInputMediaCount(), 2u);

	CU_ASSERT_EQUAL(sink.callRemoveInputMedias(), 0);
	CU_ASSERT_EQUAL(sink.getInputMediaCount(), 0u);
}


static void testSinkAddInputMediaFormatMismatch()
{
	TestElementListener l;
	/* TestSinkElement only accepts vdef_h264_byte_stream; H.265 → -ENOSYS
	 */
	CodedVideoMedia media(g_test_session->get());
	media.format = vdef_h265_byte_stream;
	TestSinkElement sink(g_test_session->get(), &l, 4);

	CU_ASSERT_EQUAL(sink.addInputMedia(&media), -ENOSYS);
}


static void testSinkAddInputMediaAudio()
{
	TestElementListener l;
	AudioMedia media(g_test_session->get());
	media.format = adef_aac_lc_16b_44100hz_stereo_adts;
	TestSinkElementAudio sink(g_test_session->get(), &l, 4);

	CU_ASSERT_EQUAL(sink.addInputMedia(&media), 0);
	CU_ASSERT_EQUAL(sink.getInputMediaCount(), 1u);
}


CU_TestInfo g_pdraw_test_sink[] = {
	{FN("testSinkAddInputMediaCoded"), testSinkAddInputMediaCoded},
	{FN("testSinkAddInputMediaDuplicate"), testSinkAddInputMediaDuplicate},
	{FN("testSinkAddBeyondMaxRejected"), testSinkAddBeyondMaxRejected},
	{FN("testSinkFindInputMediaExisting"), testSinkFindInputMediaExisting},
	{FN("testSinkFindInputMediaMissing"), testSinkFindInputMediaMissing},
	{FN("testSinkInputCountAfterMultipleAdds"),
	 testSinkInputCountAfterMultipleAdds},
	{FN("testSinkAddNullMediaReturnsError"),
	 testSinkAddNullMediaReturnsError},
	{FN("testSinkRemoveInputMedia"), testSinkRemoveInputMedia},
	{FN("testSinkGetInputMediaByIndex"), testSinkGetInputMediaByIndex},
	{FN("testSinkAddInputMediaRawVideo"), testSinkAddInputMediaRawVideo},
	{FN("testSinkAddInputMediaFormatMismatch"),
	 testSinkAddInputMediaFormatMismatch},
	{FN("testSinkAddInputMediaAudio"), testSinkAddInputMediaAudio},
	{FN("testSinkAddInputMediaTearingDown"),
	 testSinkAddInputMediaTearingDown},
	{FN("testSinkRemoveAllInputMedias"), testSinkRemoveAllInputMedias},
	CU_TEST_INFO_NULL,
};
