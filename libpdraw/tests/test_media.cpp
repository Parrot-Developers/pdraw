/**
 * Parrot Drones Audio and Video Vector library
 * libpdraw unit tests — Media classes (Tier B)
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

#define ULOG_TAG pdraw_test_media
#include "pdraw_media.hpp"
#include "test_common.h"
#include "test_fixtures.hpp"

ULOG_DECLARE_TAG(ULOG_TAG);

using namespace Pdraw;
using namespace PdrawTest;


static void testGetMediaTypeStrAllValues()
{
	struct TestParam {
		Media::Type type;
		const char *expectedStr;
	};

	static constexpr std::array<TestParam, 4> validCases{
		{{Media::Type::UNKNOWN, "UNKNOWN"},
		 {Media::Type::RAW_VIDEO, "RAW_VIDEO"},
		 {Media::Type::CODED_VIDEO, "CODED_VIDEO"},
		 {Media::Type::AUDIO, "AUDIO"}}};

	for (const auto &test : validCases) {
		CU_ASSERT_STRING_EQUAL(Media::getMediaTypeStr(test.type),
				       test.expectedStr);
	}

	Media::Type invalidType = static_cast<Media::Type>(-1);
	CU_ASSERT_PTR_NULL(Media::getMediaTypeStr(invalidType));
}


static void testCodedVideoMediaSetPsWrongEncoding()
{
	/* Default format.encoding == 0 (not H264/H265) → -EPROTO */
	CodedVideoMedia m(nullptr);
	static const uint8_t sps[] = {0x67};
	static const uint8_t pps[] = {0x68};
	CU_ASSERT_EQUAL(m.setPs(nullptr, 0, sps, sizeof(sps), pps, sizeof(pps)),
			-EPROTO);
}


static void testCodedVideoMediaGetPsWrongEncoding()
{
	/* Default format.encoding == 0 → getPs returns -EPROTO */
	CodedVideoMedia m(nullptr);
	const uint8_t *ov, *os, *op;
	size_t vs, ss, ps;
	CU_ASSERT_EQUAL(m.getPs(&ov, &vs, &os, &ss, &op, &ps), -EPROTO);
}


static void testCodedVideoMediaSetPsNullSpsRejected()
{
	/* With a valid encoding, null SPS must be rejected with -EINVAL.
	 * The null check fires before h264_get_info, so no NAL parsing
	 * is needed. */
	CodedVideoMedia m(nullptr);
	m.format.encoding = VDEF_ENCODING_H264;
	CU_ASSERT_EQUAL(m.setPs(nullptr, 0, nullptr, 0, nullptr, 0), -EINVAL);
}


static void testRawVideoMediaPublicMembers()
{
	RawVideoMedia m(nullptr);

	/* Type is set by the constructor; getType() is the accessor. */
	CU_ASSERT_EQUAL(m.getType(), Media::Type::RAW_VIDEO);

	/* Verify that the media has an auto-assigned id > 0. */
	CU_ASSERT(m.getId() > 0u);
}


static void testAudioMediaSetGetAacAsc()
{
	AudioMedia m(nullptr);
	m.format.encoding = ADEF_ENCODING_AAC_LC;

	static const uint8_t asc[] = {0x12, 0x10};
	CU_ASSERT_EQUAL(m.setAacAsc(asc, sizeof(asc)), 0);

	const uint8_t *out;
	size_t outSize;
	CU_ASSERT_EQUAL(m.getAacAsc(&out, &outSize), 0);
	CU_ASSERT_EQUAL(outSize, sizeof(asc));
	CU_ASSERT(memcmp(out, asc, sizeof(asc)) == 0);
}


static void testAudioMediaSetAacAscNullReturnsError()
{
	/* setAacAsc rejects null/empty input with -EINVAL
	 * (pdraw_media.cpp:371). */
	AudioMedia m(nullptr);
	m.format.encoding = ADEF_ENCODING_AAC_LC;
	CU_ASSERT_EQUAL(m.setAacAsc(nullptr, 0), -EINVAL);
}


static void testRawVideoMediaFillMediaInfo()
{
	RawVideoMedia m(nullptr);
	m.format = vdef_i420;

	struct pdraw_media_info info = {};
	m.fillMediaInfo(&info);

	CU_ASSERT_EQUAL(info.type, PDRAW_MEDIA_TYPE_VIDEO);
	CU_ASSERT_EQUAL(info.id, m.getId());
	CU_ASSERT_PTR_NOT_NULL(info.name);
	CU_ASSERT_EQUAL(info.video.format, VDEF_FRAME_TYPE_RAW);

	Media::cleanupMediaInfo(&info);
}


static void testCodedVideoMediaFillMediaInfo()
{
	CodedVideoMedia m(nullptr);
	m.format = vdef_h264_byte_stream;

	struct pdraw_media_info info = {};
	m.fillMediaInfo(&info);

	CU_ASSERT_EQUAL(info.type, PDRAW_MEDIA_TYPE_VIDEO);
	CU_ASSERT_EQUAL(info.id, m.getId());
	CU_ASSERT_PTR_NOT_NULL(info.name);
	CU_ASSERT_EQUAL(info.video.format, VDEF_FRAME_TYPE_CODED);
	CU_ASSERT_EQUAL(info.video.coded.format.encoding, m.format.encoding);

	Media::cleanupMediaInfo(&info);
}


static void testAudioMediaFillMediaInfo()
{
	AudioMedia m(nullptr);
	m.format.encoding = ADEF_ENCODING_AAC_LC;

	struct pdraw_media_info info = {};
	m.fillMediaInfo(&info);

	CU_ASSERT_EQUAL(info.type, PDRAW_MEDIA_TYPE_AUDIO);
	CU_ASSERT_EQUAL(info.id, m.getId());
	CU_ASSERT_PTR_NOT_NULL(info.name);

	Media::cleanupMediaInfo(&info);
}


static void testMediaSetTearingDown()
{
	RawVideoMedia m(nullptr);
	CU_ASSERT_EQUAL(m.isTearingDown(), false);
	m.setTearingDown();
	CU_ASSERT_EQUAL(m.isTearingDown(), true);
}


static void testMediaIdAutoIncrement()
{
	RawVideoMedia m1(nullptr), m2(nullptr);
	CU_ASSERT(m2.getId() > m1.getId());
}


static void testCodedVideoMediaFillMediaInfoH265()
{
	CodedVideoMedia m(nullptr);
	m.format = vdef_h265_byte_stream;

	struct pdraw_media_info info = {};
	m.fillMediaInfo(&info);

	CU_ASSERT_EQUAL(info.type, PDRAW_MEDIA_TYPE_VIDEO);
	CU_ASSERT_EQUAL(info.video.format, VDEF_FRAME_TYPE_CODED);
	CU_ASSERT_EQUAL(info.video.coded.format.encoding, VDEF_ENCODING_H265);

	Media::cleanupMediaInfo(&info);
}


static void testMediaSetClassNameOverloads()
{
	/* Local Mock class to expose the protected setClassName methods */
	class TestMediaMock : public RawVideoMedia {
	public:
		explicit TestMediaMock(Session *session) :
				RawVideoMedia(session)
		{
		}
		void exposeSetClassName(const std::string &name)
		{
			setClassName(name);
		}
		void exposeSetClassName(const char *name)
		{
			setClassName(name);
		}
	};

	TestMediaMock mockMedia(nullptr);
	unsigned int mockId = mockMedia.getId();

	/* 1. Test setClassName(const std::string &) overload */
	std::string nameStr = "StringClass";
	mockMedia.exposeSetClassName(nameStr);
	std::string expectedNameStr = "StringClass#" + std::to_string(mockId);
	CU_ASSERT_STRING_EQUAL(mockMedia.getName().c_str(),
			       expectedNameStr.c_str());

	/* 2. Test setClassName(const char *) overload */
	const char *nameChar = "CharClass";
	mockMedia.exposeSetClassName(nameChar);
	std::string expectedNameChar = "CharClass#" + std::to_string(mockId);
	CU_ASSERT_STRING_EQUAL(mockMedia.getName().c_str(),
			       expectedNameChar.c_str());
}


static void testMediaSetPathOverloads()
{
	RawVideoMedia m(nullptr);

	/* 1. Test setPath(std::string_view) overload */
	std::string_view pathView = "/u/local/video_view.mp4";
	m.setPath(pathView);
	CU_ASSERT_STRING_EQUAL(m.getPath().c_str(), "/u/local/video_view.mp4");

	/* 2. Test setPath(const char *) overload */
	const char *pathChar = "/u/local/video_char.mp4";
	m.setPath(pathChar);
	CU_ASSERT_STRING_EQUAL(m.getPath().c_str(), "/u/local/video_char.mp4");
}


CU_TestInfo g_pdraw_test_media[] = {
	{FN("testGetMediaTypeStrAllValues"), testGetMediaTypeStrAllValues},
	{FN("testCodedVideoMediaSetPsWrongEncoding"),
	 testCodedVideoMediaSetPsWrongEncoding},
	{FN("testCodedVideoMediaGetPsWrongEncoding"),
	 testCodedVideoMediaGetPsWrongEncoding},
	{FN("testCodedVideoMediaSetPsNullSpsRejected"),
	 testCodedVideoMediaSetPsNullSpsRejected},
	{FN("testRawVideoMediaPublicMembers"), testRawVideoMediaPublicMembers},
	{FN("testAudioMediaSetGetAacAsc"), testAudioMediaSetGetAacAsc},
	{FN("testAudioMediaSetAacAscNullReturnsError"),
	 testAudioMediaSetAacAscNullReturnsError},
	{FN("testRawVideoMediaFillMediaInfo"), testRawVideoMediaFillMediaInfo},
	{FN("testCodedVideoMediaFillMediaInfo"),
	 testCodedVideoMediaFillMediaInfo},
	{FN("testAudioMediaFillMediaInfo"), testAudioMediaFillMediaInfo},
	{FN("testMediaSetTearingDown"), testMediaSetTearingDown},
	{FN("testMediaIdAutoIncrement"), testMediaIdAutoIncrement},
	{FN("testCodedVideoMediaFillMediaInfoH265"),
	 testCodedVideoMediaFillMediaInfoH265},
	{FN("testMediaSetClassNameOverloads"), testMediaSetClassNameOverloads},
	{FN("testMediaSetPathOverloads"), testMediaSetPathOverloads},
	CU_TEST_INFO_NULL,
};
