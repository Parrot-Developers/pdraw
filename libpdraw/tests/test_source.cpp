/**
 * Parrot Drones Audio and Video Vector library
 * libpdraw unit tests — Source output-port management (Tier B)
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

#define ULOG_TAG pdraw_test_source
#include "test_common.h"
#include "test_fixtures.hpp"
#include "test_mocks.hpp"

#include "pdraw_media.hpp"

#include <h264/h264.h>
#include <media-buffers/mbuf_coded_video_frame.h>
#include <media-buffers/mbuf_mem_generic.h>
#include <video-defs/vdefs.h>

#include <arpa/inet.h>
#include <string.h>

ULOG_DECLARE_TAG(ULOG_TAG);

using namespace Pdraw;
using namespace PdrawTest;


static void testSourceAddOutputPortNotifiesListener()
{
	TestElementListener el;
	TestSourceListener sl;
	RawVideoMedia media(
		nullptr); /* outlives src — src dtor logs media name */
	TestSourceElement src(nullptr, &el, &sl, 4);

	int ret = src.callAddOutputPort(&media);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(sl.addedCount(), 1);
	CU_ASSERT_PTR_EQUAL(sl.lastAdded(), &media);
}


static void testSourceOutputMediaCountGrows()
{
	/* Source::addOutputPort has no duplicate guard — each call appends
	 * a new port entry; verify count reflects this. */
	TestElementListener el;
	TestSourceListener sl;
	RawVideoMedia m1(nullptr), m2(nullptr); /* outlive src */
	TestSourceElement src(nullptr, &el, &sl, 4);

	CU_ASSERT_EQUAL(src.getOutputMediaCount(), 0u);
	src.callAddOutputPort(&m1);
	CU_ASSERT_EQUAL(src.getOutputMediaCount(), 1u);
	src.callAddOutputPort(&m2);
	CU_ASSERT_EQUAL(src.getOutputMediaCount(), 2u);
}


static void testSourceAddBeyondMaxRejected()
{
	TestElementListener el;
	TestSourceListener sl;
	RawVideoMedia m1(nullptr), m2(nullptr), m3(nullptr); /* outlive src */
	TestSourceElement src(nullptr, &el, &sl, 2);

	CU_ASSERT_EQUAL(src.callAddOutputPort(&m1), 0);
	CU_ASSERT_EQUAL(src.callAddOutputPort(&m2), 0);
	int ret = src.callAddOutputPort(&m3);
	CU_ASSERT_EQUAL(ret, -ENOBUFS);
}


static void testSourceRemoveOutputPortNotifiesListener()
{
	TestElementListener el;
	TestSourceListener sl;
	TestSourceElement src(nullptr, &el, &sl, 4);
	RawVideoMedia media(nullptr);

	src.callAddOutputPort(&media);
	sl.clear();

	int ret = src.callRemoveOutputPort(&media);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(sl.removedCount(), 1);
}


static void testSourceRemoveNonExistentReturnsError()
{
	TestElementListener el;
	TestSourceListener sl;
	TestSourceElement src(nullptr, &el, &sl, 4);
	RawVideoMedia media(nullptr);

	int ret = src.callRemoveOutputPort(&media);
	CU_ASSERT_EQUAL(ret, -ENOENT);
}


static void testSourceGetOutputChannelCountEmpty()
{
	TestElementListener el;
	TestSourceListener sl;
	RawVideoMedia media(nullptr); /* outlives src */
	TestSourceElement src(nullptr, &el, &sl, 4);

	src.callAddOutputPort(&media);
	CU_ASSERT_EQUAL(src.getOutputChannelCount(&media), 0u);
}


static void testSourceFindOutputMediaAfterAdd()
{
	TestElementListener el;
	TestSourceListener sl;
	RawVideoMedia media(nullptr); /* outlives src */
	TestSourceElement src(nullptr, &el, &sl, 4);

	src.callAddOutputPort(&media);
	CU_ASSERT_PTR_EQUAL(src.findOutputMedia(&media), &media);
}


static void testSourceGetOutputMediaByIndex()
{
	TestElementListener el;
	TestSourceListener sl;
	RawVideoMedia m1(nullptr), m2(nullptr); /* outlive src */
	TestSourceElement src(nullptr, &el, &sl, 4);

	src.callAddOutputPort(&m1);
	src.callAddOutputPort(&m2);
	CU_ASSERT_PTR_EQUAL(src.getOutputMedia(0), &m1);
	CU_ASSERT_PTR_EQUAL(src.getOutputMedia(1), &m2);
	CU_ASSERT_PTR_NULL(src.getOutputMedia(2)); /* out of range */
}


static void testSourceNullListenerNoCrash()
{
	TestElementListener el;
	RawVideoMedia media(nullptr); /* outlives src */
	/* srcListener=nullptr: callAddOutputPort must not crash on the
	 * Source::mListener null check inside the wrapper. */
	TestSourceElement src(nullptr, &el, nullptr, 4);

	CU_ASSERT_EQUAL(src.callAddOutputPort(&media), 0);
	CU_ASSERT_EQUAL(src.getOutputMediaCount(), 1u);
}


static void testSourceFindOutputMediaMissing()
{
	TestElementListener el;
	TestSourceListener sl;
	RawVideoMedia media(nullptr);
	TestSourceElement src(nullptr, &el, &sl, 4);

	/* media was never added → findOutputMedia returns nullptr */
	CU_ASSERT_PTR_NULL(src.findOutputMedia(&media));
}


static void testSourceGetOutputChannelCountOutOfRange()
{
	TestElementListener el;
	TestSourceListener sl;
	RawVideoMedia media(nullptr);
	TestSourceElement src(nullptr, &el, &sl, 4);

	/* nullptr media → EINVAL → 0 */
	CU_ASSERT_EQUAL(src.getOutputChannelCount(nullptr), 0u);

	/* media with no registered port → ENOENT → 0 */
	CU_ASSERT_EQUAL(src.getOutputChannelCount(&media), 0u);
}


static void testSourceRemoveOutputPorts()
{
	TestElementListener el;
	TestSourceListener sl;
	RawVideoMedia m1(nullptr), m2(nullptr);
	TestSourceElement src(nullptr, &el, &sl, 4);

	src.callAddOutputPort(&m1);
	src.callAddOutputPort(&m2);
	CU_ASSERT_EQUAL(src.getOutputMediaCount(), 2u);

	int ret = src.callRemoveOutputPorts();
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(src.getOutputMediaCount(), 0u);
	CU_ASSERT_EQUAL(sl.removedCount(), 2);
}


static void testSourceDestroyOutputPortMemoryPool()
{
	TestElementListener el;
	TestSourceListener sl;
	RawVideoMedia media(nullptr);
	TestSourceElement src(nullptr, &el, &sl, 4);

	int ret = src.callDestroyOutputPortMemoryPool(nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	ret = src.callDestroyOutputPortMemoryPool(&media);
	CU_ASSERT_EQUAL(ret, -ENOENT);

	src.callAddOutputPort(&media);

	ret = src.callCreateOutputPortMemoryPool(&media, 2, 1024);
	CU_ASSERT_EQUAL(ret, 0);

	ret = src.callDestroyOutputPortMemoryPool(&media);
	CU_ASSERT_EQUAL(ret, 0);
}


static void testSourceOnChannelFlushedAndDrained()
{
	TestElementListener el;
	TestSourceListener sl;
	TestSourceElement src(nullptr, &el, &sl, 4);

	src.callOnChannelFlushed(nullptr);
	src.callOnChannelDrained(nullptr);
	CU_PASS("no crash on null channel");
}


static void testSourceCopyCodedVideoOutputFrame()
{
	TestElementListener el;
	TestSourceListener sl;
	TestSourceElement src(nullptr, &el, &sl, 4);

	CodedVideoMedia srcMedia(nullptr);
	srcMedia.format.encoding = VDEF_ENCODING_H264;
	srcMedia.format.data_format = VDEF_CODED_DATA_FORMAT_AVCC;
	srcMedia.info.resolution.width = 64;
	srcMedia.info.resolution.height = 64;

	CodedVideoMedia dstMedia(nullptr);
	dstMedia.format.encoding = VDEF_ENCODING_H264;
	dstMedia.format.data_format = VDEF_CODED_DATA_FORMAT_BYTE_STREAM;
	dstMedia.info.resolution.width = 64;
	dstMedia.info.resolution.height = 64;

	struct mbuf_coded_video_frame *dstFrame = nullptr;

	int ret = src.callCopyCodedVideoOutputFrame(
		nullptr, nullptr, nullptr, nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	CodedVideoMedia mismatchMedia(nullptr);
	mismatchMedia.format.encoding = VDEF_ENCODING_HEVC;
	ret = src.callCopyCodedVideoOutputFrame(
		&srcMedia,
		(struct mbuf_coded_video_frame *)1,
		&mismatchMedia,
		&dstFrame);
	CU_ASSERT_EQUAL(ret, -EINVAL);
}


/* A single-NALU frame whose packed buffer is laid out exactly like
 * `dataFormat` on the wire (4-byte big-endian length prefix for AVCC, 4-byte
 * 00 00 00 01 start code for byte-stream) -- unlike a frame's own recorded
 * `vdef_coded_frame::format` field, this raw byte layout is what
 * h264_avcc_to_byte_stream()/h264_byte_stream_to_avcc() and their H.265
 * counterparts h265_hvcc_to_byte_stream()/h265_byte_stream_to_hvcc() (invoked
 * by Source::copyCodedVideoOutputFrame(), pdraw_source.cpp) actually read and
 * rewrite in place, so it must be real to exercise those conversions.
 * `encoding` defaults to H264; pass VDEF_ENCODING_H265 to build the HEVC
 * equivalent (only the NAL unit type field of the union differs, the wire
 * layout and length-prefix rewrite are otherwise identical). */
static struct mbuf_coded_video_frame *
makeSingleNaluCodedFrame(enum vdef_coded_data_format dataFormat,
			 enum vdef_encoding encoding = VDEF_ENCODING_H264)
{
	static const uint8_t kPayload[4] = {0x65, 0x88, 0x84, 0x00};

	struct vdef_coded_frame frameInfo = {};
	frameInfo.format.encoding = encoding;
	frameInfo.format.data_format = dataFormat;
	frameInfo.info.timescale = 1000000;
	frameInfo.info.resolution.width = 64;
	frameInfo.info.resolution.height = 64;
	frameInfo.type = VDEF_CODED_FRAME_TYPE_IDR;

	struct mbuf_coded_video_frame *frame = nullptr;
	int ret = mbuf_coded_video_frame_new(&frameInfo, &frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	size_t total = 4 + sizeof(kPayload);
	struct mbuf_mem *mem = nullptr;
	ret = mbuf_mem_generic_new(total, &mem);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	uint8_t *data = nullptr;
	size_t capacity = 0;
	ret = mbuf_mem_get_data(mem, (void **)&data, &capacity);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	if (dataFormat == VDEF_CODED_DATA_FORMAT_AVCC) {
		uint32_t len_be = htonl((uint32_t)sizeof(kPayload));
		memcpy(data, &len_be, 4);
	} else {
		uint32_t start_code = htonl(0x00000001);
		memcpy(data, &start_code, 4);
	}
	memcpy(data + 4, kPayload, sizeof(kPayload));

	struct vdef_nalu nalu = {};
	nalu.size = total;
	if (encoding == VDEF_ENCODING_H265)
		nalu.h265.type = H265_NALU_TYPE_IDR_W_RADL;
	else
		nalu.h264.type = H264_NALU_TYPE_SLICE_IDR;
	ret = mbuf_coded_video_frame_add_nalu(frame, mem, 0, &nalu);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	mbuf_mem_unref(mem); /* the frame holds its own ref via add_nalu */

	ret = mbuf_coded_video_frame_finalize(frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	return frame;
}


static void testSourceCopyCodedVideoOutputFrameSameFormatIsVerbatim()
{
	TestElementListener el;
	TestSourceListener sl;
	TestSourceElement src(nullptr, &el, &sl, 4);

	CodedVideoMedia srcMedia(nullptr);
	srcMedia.format.encoding = VDEF_ENCODING_H264;
	srcMedia.format.data_format = VDEF_CODED_DATA_FORMAT_AVCC;

	CodedVideoMedia dstMedia(nullptr);
	dstMedia.format.encoding = VDEF_ENCODING_H264;
	dstMedia.format.data_format = VDEF_CODED_DATA_FORMAT_AVCC;

	int ret = src.callAddOutputPort(&dstMedia);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	ret = src.callCreateOutputPortMemoryPool(&dstMedia, 1, 4096);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	struct mbuf_coded_video_frame *srcFrame =
		makeSingleNaluCodedFrame(VDEF_CODED_DATA_FORMAT_AVCC);
	struct mbuf_coded_video_frame *dstFrame = nullptr;

	ret = src.callCopyCodedVideoOutputFrame(
		&srcMedia, srcFrame, &dstMedia, &dstFrame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(dstFrame);

	const void *srcData = nullptr, *dstData = nullptr;
	size_t srcLen = 0, dstLen = 0;
	CU_ASSERT_EQUAL_FATAL(mbuf_coded_video_frame_get_packed_buffer(
				      srcFrame, &srcData, &srcLen),
			      0);
	CU_ASSERT_EQUAL_FATAL(mbuf_coded_video_frame_get_packed_buffer(
				      dstFrame, &dstData, &dstLen),
			      0);
	/* Same encoding + data_format on both sides: bytes are copied as-is,
	 * no AVCC<->byte-stream rewrite is attempted. */
	CU_ASSERT_EQUAL(dstLen, srcLen);
	CU_ASSERT_EQUAL(memcmp(dstData, srcData, srcLen), 0);
	mbuf_coded_video_frame_release_packed_buffer(dstFrame, dstData);
	mbuf_coded_video_frame_release_packed_buffer(srcFrame, srcData);

	mbuf_coded_video_frame_unref(dstFrame);
	mbuf_coded_video_frame_unref(srcFrame);
	src.callDestroyOutputPortMemoryPool(&dstMedia);
	/* Must run before dstMedia goes out of scope: src (declared before
	 * srcMedia/dstMedia) outlives them and would otherwise have its
	 * destructor (Source::~Source() -> removeOutputPorts()) try to log
	 * the name of an already-destroyed media -- a heap-use-after-free
	 * confirmed by a real ASan run. */
	CU_ASSERT_EQUAL(src.callRemoveOutputPort(&dstMedia), 0);
}


static void testSourceCopyCodedVideoOutputFrameConvertsAvccToByteStream()
{
	TestElementListener el;
	TestSourceListener sl;
	TestSourceElement src(nullptr, &el, &sl, 4);

	CodedVideoMedia srcMedia(nullptr);
	srcMedia.format.encoding = VDEF_ENCODING_H264;
	srcMedia.format.data_format = VDEF_CODED_DATA_FORMAT_AVCC;

	CodedVideoMedia dstMedia(nullptr);
	dstMedia.format.encoding = VDEF_ENCODING_H264;
	dstMedia.format.data_format = VDEF_CODED_DATA_FORMAT_BYTE_STREAM;

	/* getCodedVideoOutputMemory() (pdraw_source.cpp) needs a real pool
	 * behind dstMedia's output port to hand back destination memory. */
	int ret = src.callAddOutputPort(&dstMedia);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	ret = src.callCreateOutputPortMemoryPool(&dstMedia, 1, 4096);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	struct mbuf_coded_video_frame *srcFrame =
		makeSingleNaluCodedFrame(VDEF_CODED_DATA_FORMAT_AVCC);
	struct mbuf_coded_video_frame *dstFrame = nullptr;

	ret = src.callCopyCodedVideoOutputFrame(
		&srcMedia, srcFrame, &dstMedia, &dstFrame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(dstFrame);

	struct vdef_coded_frame dstInfo = {};
	CU_ASSERT_EQUAL_FATAL(
		mbuf_coded_video_frame_get_frame_info(dstFrame, &dstInfo), 0);
	CU_ASSERT_TRUE(
		vdef_coded_format_cmp(&dstInfo.format, &dstMedia.format));

	const void *dstData = nullptr;
	size_t dstLen = 0;
	CU_ASSERT_EQUAL_FATAL(mbuf_coded_video_frame_get_packed_buffer(
				      dstFrame, &dstData, &dstLen),
			      0);
	CU_ASSERT_EQUAL_FATAL(dstLen, 8u);
	/* h264_avcc_to_byte_stream() (libh264) replaces the 4-byte AVCC
	 * length prefix with a start code in place, same length. */
	static const uint8_t kStartCode[4] = {0x00, 0x00, 0x00, 0x01};
	CU_ASSERT_EQUAL(memcmp(dstData, kStartCode, 4), 0);
	mbuf_coded_video_frame_release_packed_buffer(dstFrame, dstData);

	mbuf_coded_video_frame_unref(dstFrame);
	mbuf_coded_video_frame_unref(srcFrame);
	src.callDestroyOutputPortMemoryPool(&dstMedia);
	/* Must run before dstMedia goes out of scope: src (declared before
	 * srcMedia/dstMedia) outlives them and would otherwise have its
	 * destructor (Source::~Source() -> removeOutputPorts()) try to log
	 * the name of an already-destroyed media -- a heap-use-after-free
	 * confirmed by a real ASan run. */
	CU_ASSERT_EQUAL(src.callRemoveOutputPort(&dstMedia), 0);
}


static void testSourceCopyCodedVideoOutputFrameConvertsByteStreamToAvcc()
{
	TestElementListener el;
	TestSourceListener sl;
	TestSourceElement src(nullptr, &el, &sl, 4);

	CodedVideoMedia srcMedia(nullptr);
	srcMedia.format.encoding = VDEF_ENCODING_H264;
	srcMedia.format.data_format = VDEF_CODED_DATA_FORMAT_BYTE_STREAM;

	CodedVideoMedia dstMedia(nullptr);
	dstMedia.format.encoding = VDEF_ENCODING_H264;
	dstMedia.format.data_format = VDEF_CODED_DATA_FORMAT_AVCC;

	int ret = src.callAddOutputPort(&dstMedia);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	ret = src.callCreateOutputPortMemoryPool(&dstMedia, 1, 4096);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	struct mbuf_coded_video_frame *srcFrame =
		makeSingleNaluCodedFrame(VDEF_CODED_DATA_FORMAT_BYTE_STREAM);
	struct mbuf_coded_video_frame *dstFrame = nullptr;

	ret = src.callCopyCodedVideoOutputFrame(
		&srcMedia, srcFrame, &dstMedia, &dstFrame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(dstFrame);

	const void *dstData = nullptr;
	size_t dstLen = 0;
	CU_ASSERT_EQUAL_FATAL(mbuf_coded_video_frame_get_packed_buffer(
				      dstFrame, &dstData, &dstLen),
			      0);
	CU_ASSERT_EQUAL_FATAL(dstLen, 8u);
	/* h264_byte_stream_to_avcc() (libh264) replaces the start code with
	 * a 4-byte big-endian NALU length in place, same length. */
	uint32_t len_be = 0;
	memcpy(&len_be, dstData, 4);
	CU_ASSERT_EQUAL(ntohl(len_be), 4u);
	mbuf_coded_video_frame_release_packed_buffer(dstFrame, dstData);

	mbuf_coded_video_frame_unref(dstFrame);
	mbuf_coded_video_frame_unref(srcFrame);
	src.callDestroyOutputPortMemoryPool(&dstMedia);
	/* Must run before dstMedia goes out of scope: src (declared before
	 * srcMedia/dstMedia) outlives them and would otherwise have its
	 * destructor (Source::~Source() -> removeOutputPorts()) try to log
	 * the name of an already-destroyed media -- a heap-use-after-free
	 * confirmed by a real ASan run. */
	CU_ASSERT_EQUAL(src.callRemoveOutputPort(&dstMedia), 0);
}


/* H.265 equivalent of
 * testSourceCopyCodedVideoOutputFrameConvertsAvccToByteStream: the
 * AVCC<->byte-stream switch in Source::copyCodedVideoOutputFrame()
 * (pdraw_source.cpp) has a separate case per encoding
 * (h264_avcc_to_byte_stream() vs h265_hvcc_to_byte_stream()) -- only the H264
 * case was ever exercised before this test. */
static void testSourceCopyCodedVideoOutputFrameConvertsHvccToByteStreamH265()
{
	TestElementListener el;
	TestSourceListener sl;
	TestSourceElement src(nullptr, &el, &sl, 4);

	CodedVideoMedia srcMedia(nullptr);
	srcMedia.format.encoding = VDEF_ENCODING_H265;
	srcMedia.format.data_format = VDEF_CODED_DATA_FORMAT_AVCC;

	CodedVideoMedia dstMedia(nullptr);
	dstMedia.format.encoding = VDEF_ENCODING_H265;
	dstMedia.format.data_format = VDEF_CODED_DATA_FORMAT_BYTE_STREAM;

	int ret = src.callAddOutputPort(&dstMedia);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	ret = src.callCreateOutputPortMemoryPool(&dstMedia, 1, 4096);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	struct mbuf_coded_video_frame *srcFrame = makeSingleNaluCodedFrame(
		VDEF_CODED_DATA_FORMAT_AVCC, VDEF_ENCODING_H265);
	struct mbuf_coded_video_frame *dstFrame = nullptr;

	ret = src.callCopyCodedVideoOutputFrame(
		&srcMedia, srcFrame, &dstMedia, &dstFrame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(dstFrame);

	const void *dstData = nullptr;
	size_t dstLen = 0;
	CU_ASSERT_EQUAL_FATAL(mbuf_coded_video_frame_get_packed_buffer(
				      dstFrame, &dstData, &dstLen),
			      0);
	CU_ASSERT_EQUAL_FATAL(dstLen, 8u);
	/* h265_hvcc_to_byte_stream() (libh265) replaces the 4-byte HVCC length
	 * prefix with a start code in place, same length -- identical layout
	 * to h264_avcc_to_byte_stream(). */
	static const uint8_t kStartCode[4] = {0x00, 0x00, 0x00, 0x01};
	CU_ASSERT_EQUAL(memcmp(dstData, kStartCode, 4), 0);
	mbuf_coded_video_frame_release_packed_buffer(dstFrame, dstData);

	mbuf_coded_video_frame_unref(dstFrame);
	mbuf_coded_video_frame_unref(srcFrame);
	src.callDestroyOutputPortMemoryPool(&dstMedia);
	/* Must run before dstMedia goes out of scope: src (declared before
	 * srcMedia/dstMedia) outlives them and would otherwise have its
	 * destructor (Source::~Source() -> removeOutputPorts()) try to log
	 * the name of an already-destroyed media -- a heap-use-after-free
	 * confirmed by a real ASan run. */
	CU_ASSERT_EQUAL(src.callRemoveOutputPort(&dstMedia), 0);
}


static void testSourceCopyCodedVideoOutputFrameConvertsByteStreamToHvccH265()
{
	TestElementListener el;
	TestSourceListener sl;
	TestSourceElement src(nullptr, &el, &sl, 4);

	CodedVideoMedia srcMedia(nullptr);
	srcMedia.format.encoding = VDEF_ENCODING_H265;
	srcMedia.format.data_format = VDEF_CODED_DATA_FORMAT_BYTE_STREAM;

	CodedVideoMedia dstMedia(nullptr);
	dstMedia.format.encoding = VDEF_ENCODING_H265;
	dstMedia.format.data_format = VDEF_CODED_DATA_FORMAT_AVCC;

	int ret = src.callAddOutputPort(&dstMedia);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	ret = src.callCreateOutputPortMemoryPool(&dstMedia, 1, 4096);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	struct mbuf_coded_video_frame *srcFrame = makeSingleNaluCodedFrame(
		VDEF_CODED_DATA_FORMAT_BYTE_STREAM, VDEF_ENCODING_H265);
	struct mbuf_coded_video_frame *dstFrame = nullptr;

	ret = src.callCopyCodedVideoOutputFrame(
		&srcMedia, srcFrame, &dstMedia, &dstFrame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(dstFrame);

	const void *dstData = nullptr;
	size_t dstLen = 0;
	CU_ASSERT_EQUAL_FATAL(mbuf_coded_video_frame_get_packed_buffer(
				      dstFrame, &dstData, &dstLen),
			      0);
	CU_ASSERT_EQUAL_FATAL(dstLen, 8u);
	/* h265_byte_stream_to_hvcc() (libh265) replaces the start code with a
	 * 4-byte big-endian NALU length in place, same length -- identical
	 * layout to h264_byte_stream_to_avcc(). */
	uint32_t len_be = 0;
	memcpy(&len_be, dstData, 4);
	CU_ASSERT_EQUAL(ntohl(len_be), 4u);
	mbuf_coded_video_frame_release_packed_buffer(dstFrame, dstData);

	mbuf_coded_video_frame_unref(dstFrame);
	mbuf_coded_video_frame_unref(srcFrame);
	src.callDestroyOutputPortMemoryPool(&dstMedia);
	/* Must run before dstMedia goes out of scope: src (declared before
	 * srcMedia/dstMedia) outlives them and would otherwise have its
	 * destructor (Source::~Source() -> removeOutputPorts()) try to log
	 * the name of an already-destroyed media -- a heap-use-after-free
	 * confirmed by a real ASan run. */
	CU_ASSERT_EQUAL(src.callRemoveOutputPort(&dstMedia), 0);
}


static void testSourceCopyCodedVideoOutputFrameRawNaluUnsupported()
{
	TestElementListener el;
	TestSourceListener sl;
	TestSourceElement src(nullptr, &el, &sl, 4);

	CodedVideoMedia srcMedia(nullptr);
	srcMedia.format.encoding = VDEF_ENCODING_H264;
	srcMedia.format.data_format = VDEF_CODED_DATA_FORMAT_RAW_NALU;

	CodedVideoMedia dstMedia(nullptr);
	dstMedia.format.encoding = VDEF_ENCODING_H264;
	dstMedia.format.data_format = VDEF_CODED_DATA_FORMAT_AVCC;

	int ret = src.callAddOutputPort(&dstMedia);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	ret = src.callCreateOutputPortMemoryPool(&dstMedia, 1, 4096);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	/* Byte layout is irrelevant here: Source::copyCodedVideoOutputFrame()
	 * rejects any raw_nalu<->other conversion (pdraw_source.cpp) purely
	 * based on srcMedia/dstMedia->format.data_format, before ever
	 * touching the packed buffer bytes. */
	struct mbuf_coded_video_frame *srcFrame =
		makeSingleNaluCodedFrame(VDEF_CODED_DATA_FORMAT_RAW_NALU);
	struct mbuf_coded_video_frame *dstFrame = nullptr;

	ret = src.callCopyCodedVideoOutputFrame(
		&srcMedia, srcFrame, &dstMedia, &dstFrame);
	CU_ASSERT_EQUAL(ret, -ENOSYS);

	if (dstFrame != nullptr)
		mbuf_coded_video_frame_unref(dstFrame);
	mbuf_coded_video_frame_unref(srcFrame);
	src.callDestroyOutputPortMemoryPool(&dstMedia);
	/* Must run before dstMedia goes out of scope: src (declared before
	 * srcMedia/dstMedia) outlives them and would otherwise have its
	 * destructor (Source::~Source() -> removeOutputPorts()) try to log
	 * the name of an already-destroyed media -- a heap-use-after-free
	 * confirmed by a real ASan run. */
	CU_ASSERT_EQUAL(src.callRemoveOutputPort(&dstMedia), 0);
}


CU_TestInfo g_pdraw_test_source[] = {
	{FN("testSourceAddOutputPortNotifiesListener"),
	 testSourceAddOutputPortNotifiesListener},
	{FN("testSourceOutputMediaCountGrows"),
	 testSourceOutputMediaCountGrows},
	{FN("testSourceAddBeyondMaxRejected"), testSourceAddBeyondMaxRejected},
	{FN("testSourceRemoveOutputPortNotifiesListener"),
	 testSourceRemoveOutputPortNotifiesListener},
	{FN("testSourceRemoveNonExistentReturnsError"),
	 testSourceRemoveNonExistentReturnsError},
	{FN("testSourceGetOutputChannelCountEmpty"),
	 testSourceGetOutputChannelCountEmpty},
	{FN("testSourceFindOutputMediaAfterAdd"),
	 testSourceFindOutputMediaAfterAdd},
	{FN("testSourceGetOutputMediaByIndex"),
	 testSourceGetOutputMediaByIndex},
	{FN("testSourceGetOutputChannelCountOutOfRange"),
	 testSourceGetOutputChannelCountOutOfRange},
	{FN("testSourceNullListenerNoCrash"), testSourceNullListenerNoCrash},
	{FN("testSourceFindOutputMediaMissing"),
	 testSourceFindOutputMediaMissing},
	{FN("testSourceRemoveOutputPorts"), testSourceRemoveOutputPorts},
	{FN("testSourceDestroyOutputPortMemoryPool"),
	 testSourceDestroyOutputPortMemoryPool},
	{FN("testSourceOnChannelFlushedAndDrained"),
	 testSourceOnChannelFlushedAndDrained},
	{FN("testSourceCopyCodedVideoOutputFrame"),
	 testSourceCopyCodedVideoOutputFrame},
	{FN("testSourceCopyCodedVideoOutputFrameSameFormatIsVerbatim"),
	 testSourceCopyCodedVideoOutputFrameSameFormatIsVerbatim},
	{FN("testSourceCopyCodedVideoOutputFrameConvertsAvccToByteStream"),
	 testSourceCopyCodedVideoOutputFrameConvertsAvccToByteStream},
	{FN("testSourceCopyCodedVideoOutputFrameConvertsByteStreamToAvcc"),
	 testSourceCopyCodedVideoOutputFrameConvertsByteStreamToAvcc},
	{FN("testSourceCopyCodedVideoOutputFrameConvertsHvccToByteStreamH265"),
	 testSourceCopyCodedVideoOutputFrameConvertsHvccToByteStreamH265},
	{FN("testSourceCopyCodedVideoOutputFrameConvertsByteStreamToHvccH265"),
	 testSourceCopyCodedVideoOutputFrameConvertsByteStreamToHvccH265},
	{FN("testSourceCopyCodedVideoOutputFrameRawNaluUnsupported"),
	 testSourceCopyCodedVideoOutputFrameRawNaluUnsupported},
	CU_TEST_INFO_NULL,
};
