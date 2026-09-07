/**
 * Parrot Drones Audio and Video Vector library
 * libpdraw unit tests — RtmpStreamMuxer RTMP signaling pipeline (Tier B)
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

#define ULOG_TAG pdraw_test_pipeline_muxer_stream_rtmp
#include "mock_librtmp.hpp"
#include "test_api_common.hpp"
#include "test_common.h"
#include "test_fixtures.hpp"

#define private public
#include "pdraw_muxer.hpp"
#include "pdraw_muxer_stream_rtmp.hpp"
#undef private

#include <media-buffers/mbuf_coded_video_frame.h>
#include <media-buffers/mbuf_mem_generic.h>

#include <string>
#include <vector>

ULOG_DECLARE_TAG(ULOG_TAG);

using namespace Pdraw;
using namespace PdrawTest;

static const uint8_t kH264Sps[] = {
	0x67, 0x64, 0x00, 0x28, 0xAC, 0xD9, 0x80, 0x78, 0x06, 0x5B, 0x01, 0x10,
	0x00, 0x00, 0x3E, 0x90, 0x00, 0x0B, 0xB8, 0x08, 0xF1, 0x83, 0x19, 0xA0};
static const uint8_t kH264Pps[] = {0x68, 0xE9, 0x78, 0xF3, 0xC8, 0xF0};

namespace {

class RtmpMuxerListener : public IPdraw::IMuxer::Listener {
public:
	bool mGotConnected = false;
	bool mGotDisconnected = false;
	bool mGotCloseResponse = false;
	bool mGotUnrecoverableError = false;
	int mCloseStatus = -1;

	void onMuxerConnectionStateChanged(
		IPdraw *pdraw,
		IPdraw::IMuxer *muxer,
		enum pdraw_muxer_connection_state state,
		enum pdraw_muxer_disconnection_reason reason) override
	{
		if (state == PDRAW_MUXER_CONNECTION_STATE_CONNECTED) {
			mGotConnected = true;
		} else if (state == PDRAW_MUXER_CONNECTION_STATE_DISCONNECTED) {
			mGotDisconnected = true;
		}
	}

	void onMuxerMediaReady(IPdraw *pdraw,
			       IPdraw::IMuxer *muxer,
			       const char *media_name,
			       const struct iovec *iov,
			       int iov_len) override
	{
	}

	void onMuxerMediaSaved(IPdraw *pdraw,
			       IPdraw::IMuxer *muxer,
			       const char *media_name) override
	{
	}

	void onMuxerUnrecoverableError(IPdraw *pdraw,
				       IPdraw::IMuxer *muxer,
				       int err) override
	{
		mGotUnrecoverableError = true;
	}

	void muxerCloseResponse(IPdraw *pdraw,
				IPdraw::IMuxer *muxer,
				int status) override
	{
		mGotCloseResponse = true;
		mCloseStatus = status;
	}
};

class TestPdrawListener : public StubPdrawListener {
public:
	struct Added {
		unsigned int id;
		enum pdraw_media_type type;
		enum vdef_frame_type videoFormat;
	};

	void onMediaAdded(IPdraw * /*p*/,
			  const struct pdraw_media_info *info,
			  void * /*u*/) override
	{
		mMediaId = info->id;
		mGotMediaAdded = true;
		Added a = {};
		a.id = info->id;
		a.type = info->type;
		if (info->type == PDRAW_MEDIA_TYPE_VIDEO)
			a.videoFormat = info->video.format;
		mAdded.push_back(a);
	}

	/* First raw (uncompressed) video media added, or nullptr if none --
	 * used to get the id of a media that is deliberately NOT a
	 * CodedVideoMedia, to exercise the dynamic_cast<CodedVideoMedia *>
	 * rejection path in RtmpStreamMuxer::addInputMedia(). */
	const Added *findRawVideoMedia() const
	{
		for (const auto &a : mAdded) {
			if (a.type == PDRAW_MEDIA_TYPE_VIDEO &&
			    a.videoFormat == VDEF_FRAME_TYPE_RAW)
				return &a;
		}
		return nullptr;
	}

	/* nth (0-indexed) coded video media added -- used to get the id of a
	 * second coded video source's media, to exercise the "only 1 video
	 * media supported" rejection path in
	 * RtmpStreamMuxer::addInputMedia(). */
	const Added *findCodedVideoMedia(size_t n) const
	{
		size_t count = 0;
		for (const auto &a : mAdded) {
			if (a.type == PDRAW_MEDIA_TYPE_VIDEO &&
			    a.videoFormat == VDEF_FRAME_TYPE_CODED) {
				if (count == n)
					return &a;
				count++;
			}
		}
		return nullptr;
	}

	std::vector<Added> mAdded;
	unsigned int mMediaId = 0;
	bool mGotMediaAdded = false;
};

class TrackingCodedVideoSourceListener
		: public IPdraw::ICodedVideoSource::Listener {
public:
	bool mGotFlushed = false;
	bool mGotDrained = false;

	void onCodedVideoSourceFlushed(
		IPdraw * /*p*/,
		IPdraw::ICodedVideoSource * /*source*/) override
	{
		mGotFlushed = true;
	}

	void onCodedVideoSourceDrained(
		IPdraw * /*p*/,
		IPdraw::ICodedVideoSource * /*source*/) override
	{
		mGotDrained = true;
	}
};

} /* anonymous namespace */

static IPdraw::IMuxer *
openRtmpStreamMuxer(IPdraw *session,
		    TestPompLoop &loop,
		    const std::string &url,
		    RtmpMuxerListener *muxerListener,
		    IPdraw::ICodedVideoSource **outSource,
		    IPdraw::ICodedVideoSource::Listener *srcListener,
		    TestPdrawListener *sessionListener)
{
	struct pdraw_muxer_params muxerParams = {};

	IPdraw::IMuxer *muxer = nullptr;
	int ret =
		session->createMuxer(url, &muxerParams, muxerListener, &muxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(muxer);

	struct pdraw_video_source_params srcParams = {};
	srcParams.video.format = VDEF_FRAME_TYPE_CODED;
	srcParams.video.coded.format = vdef_h264_avcc;
	memcpy(srcParams.video.coded.h264.sps, kH264Sps, sizeof(kH264Sps));
	srcParams.video.coded.h264.spslen = sizeof(kH264Sps);
	memcpy(srcParams.video.coded.h264.pps, kH264Pps, sizeof(kH264Pps));
	srcParams.video.coded.h264.ppslen = sizeof(kH264Pps);

	ret = session->createCodedVideoSource(
		&srcParams, srcListener, outSource);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(*outSource);

	if (sessionListener) {
		bool gotMedia = loop.pumpUntil(
			[sessionListener]() {
				return sessionListener->mGotMediaAdded;
			},
			1000);
		CU_ASSERT_TRUE_FATAL(gotMedia);

		ret = muxer->addMedia(sessionListener->mMediaId, nullptr);
		CU_ASSERT_EQUAL(ret, 0);
	}

	return muxer;
}

static void closeRtmpStreamMuxer(IPdraw::IMuxer *muxer,
				 TestPompLoop &loop,
				 RtmpMuxerListener *listener)
{
	auto muxerOwner = std::unique_ptr<IPdraw::IMuxer>(muxer);
	int ret = muxer->close();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotClose = loop.pumpUntil(
		[listener]() { return listener->mGotCloseResponse; }, 10000);
	CU_ASSERT_TRUE(gotClose);
	CU_ASSERT_EQUAL(listener->mCloseStatus, 0);

	/* muxerCloseResponse() is dispatched via an idle callback queued
	 * BEFORE the Muxer element's STOPPED state-change idle (see
	 * Muxer::completeStop(), pdraw_muxer.cpp), which itself queues
	 * Session::asyncElementDelete() as a THIRD, later idle. gotClose
	 * becoming true above only proves the first of these three ran --
	 * keep pumping past it so the element is actually deleted (running
	 * MuxerWrapper::clearElement()) while muxerOwner is still alive,
	 * instead of just letting .reset() below destroy the wrapper first. */
	bool gotClearElement = loop.pumpUntil(
		[muxer]() {
			return static_cast<MuxerWrapper *>(muxer)->getMuxer() ==
			       nullptr;
		},
		10000);
	CU_ASSERT_TRUE(gotClearElement);

	muxerOwner.reset();
}

static void testRtmpStreamMuxerSuccess()
{
	librtmp_mock_reset();
	g_librtmp_mock.trigger_connection_success_async = true;

	TestPompLoop loop;
	TestPdrawListener sessionListener;
	TestSession testSession(&loop, &sessionListener);
	IPdraw *session = testSession.get();

	std::string url = "rtmp://127.0.0.1:1935/live/stream";

	RtmpMuxerListener muxerListener;
	TrackingCodedVideoSourceListener srcListener;
	IPdraw::ICodedVideoSource *source = nullptr;

	IPdraw::IMuxer *muxer = openRtmpStreamMuxer(session,
						    loop,
						    url,
						    &muxerListener,
						    &source,
						    &srcListener,
						    &sessionListener);
	auto sourceOwner = std::unique_ptr<IPdraw::ICodedVideoSource>(source);

	/* Wait for connected callback */
	bool gotConnected = loop.pumpUntil(
		[&muxerListener]() { return muxerListener.mGotConnected; },
		5000);
	CU_ASSERT_TRUE(gotConnected);

	/* Test dynamic parameters set & get */
	struct pdraw_muxer_dyn_params dynParams = {};
	dynParams.socket_tx_buffer_size = 4096;
	int ret = muxer->setDynParams(&dynParams);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(g_librtmp_mock.last_txbuf_size, 4096);

	struct pdraw_muxer_dyn_params readParams = {};
	ret = muxer->getDynParams(&readParams);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(readParams.socket_tx_buffer_size, 4096);

	/* Test peer bandwidth callback */
	librtmp_mock_trigger_peer_bw(123456);

	/* Push one packed frame */
	struct mbuf_coded_video_frame_queue *queue = source->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(queue);

	struct vdef_coded_frame frameInfo = {};
	frameInfo.format = vdef_h264_avcc;
	frameInfo.info.resolution.width = 1920;
	frameInfo.info.resolution.height = 800;
	frameInfo.info.bit_depth = 8;
	frameInfo.info.timescale = 1000000;
	frameInfo.info.timestamp = 0;
	frameInfo.type = VDEF_CODED_FRAME_TYPE_IDR;

	struct mbuf_coded_video_frame *frame = nullptr;
	ret = mbuf_coded_video_frame_new(&frameInfo, &frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	struct mbuf_mem *mem = nullptr;
	ret = mbuf_mem_generic_new(100, &mem);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	struct vdef_nalu nalu = {};
	nalu.size = 100;
	nalu.h264.type = H264_NALU_TYPE_SLICE_IDR;
	ret = mbuf_coded_video_frame_add_nalu(frame, mem, 0, &nalu);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	mbuf_mem_unref(mem);

	ret = mbuf_coded_video_frame_finalize(frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	ret = mbuf_coded_video_frame_queue_push(queue, frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	mbuf_coded_video_frame_unref(frame);

	/* Process queue and wait for delivery */
	bool delivered = loop.pumpUntil(
		[]() { return g_librtmp_mock.send_video_frame_call_count > 0; },
		2000);
	CU_ASSERT_TRUE(delivered);

	/* Stats roundtrip including bandwidth check */
	struct pdraw_muxer_stats stats = {};
	ret = muxer->getStats(&stats);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(stats.type, PDRAW_MUXER_TYPE_RTMP);
	CU_ASSERT_EQUAL(stats.rtmp.max_peer_bw, 123456);

	/* Cast to RtmpStreamMuxer to test internal callbacks */
	MuxerWrapper *wrapper = static_cast<MuxerWrapper *>(muxer);
	RtmpStreamMuxer *rtmpMuxer =
		static_cast<RtmpStreamMuxer *>(wrapper->getMuxer());
	CU_ASSERT_PTR_NOT_NULL_FATAL(rtmpMuxer);

	/* Test internal onFakeAudioTimer() */
	int prev_audio_sends = g_librtmp_mock.send_audio_data_call_count;
	rtmpMuxer->onFakeAudioTimer();
	CU_ASSERT_TRUE(g_librtmp_mock.send_audio_data_call_count >
		       prev_audio_sends);

	/* Close (but don't delete unique_ptr yet to test setter when client is
	 * null) */
	auto muxerOwner = std::unique_ptr<IPdraw::IMuxer>(muxer);
	ret = muxer->close();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotClose = loop.pumpUntil(
		[&muxerListener]() { return muxerListener.mGotCloseResponse; },
		10000);
	CU_ASSERT_TRUE(gotClose);
	CU_ASSERT_EQUAL(muxerListener.mCloseStatus, 0);

	/* Test internalStop & internalStart with socket tx buffer size != 0
	 * Since the element has closed successfully, the state is STOPPED,
	 * so internalStop will not trigger reconnection scheduling. */
	rtmpMuxer->mSocketTxBufferSize = 4096;
	ret = rtmpMuxer->internalStart();
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(g_librtmp_mock.last_txbuf_size, 4096);

	ret = rtmpMuxer->internalStop();
	CU_ASSERT_EQUAL(ret, 0);

	/* Test setDynParams when mRtmpClient is nullptr (after closing) */
	ret = rtmpMuxer->setDynParams(&dynParams);
	CU_ASSERT_EQUAL(ret, 0);

	sourceOwner.reset();
	muxerOwner.reset();
}

static void testRtmpStreamMuxerUnpacked()
{
	librtmp_mock_reset();
	g_librtmp_mock.trigger_connection_success_async = true;

	TestPompLoop loop;
	TestPdrawListener sessionListener;
	TestSession testSession(&loop, &sessionListener);
	IPdraw *session = testSession.get();

	std::string url = "rtmp://127.0.0.1:1935/live/stream";

	RtmpMuxerListener muxerListener;
	TrackingCodedVideoSourceListener srcListener;
	IPdraw::ICodedVideoSource *source = nullptr;

	IPdraw::IMuxer *muxer = openRtmpStreamMuxer(session,
						    loop,
						    url,
						    &muxerListener,
						    &source,
						    &srcListener,
						    &sessionListener);
	auto sourceOwner = std::unique_ptr<IPdraw::ICodedVideoSource>(source);

	/* Wait for connected callback */
	bool gotConnected = loop.pumpUntil(
		[&muxerListener]() { return muxerListener.mGotConnected; },
		5000);
	CU_ASSERT_TRUE(gotConnected);

	/* Push one non-packed frame (multiple NALUs in separate mem buffers) */
	struct mbuf_coded_video_frame_queue *queue = source->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(queue);

	struct vdef_coded_frame frameInfo = {};
	frameInfo.format = vdef_h264_avcc;
	frameInfo.info.resolution.width = 1920;
	frameInfo.info.resolution.height = 800;
	frameInfo.info.bit_depth = 8;
	frameInfo.info.timescale = 1000000;
	frameInfo.info.timestamp = 0;
	frameInfo.type = VDEF_CODED_FRAME_TYPE_IDR;

	struct mbuf_coded_video_frame *frame = nullptr;
	int ret = mbuf_coded_video_frame_new(&frameInfo, &frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	/* SPS NALU */
	struct mbuf_mem *mem1 = nullptr;
	ret = mbuf_mem_generic_new(sizeof(kH264Sps), &mem1);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	void *data1 = nullptr;
	size_t capacity1 = 0;
	ret = mbuf_mem_get_data(mem1, &data1, &capacity1);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	memcpy(data1, kH264Sps, sizeof(kH264Sps));

	struct vdef_nalu nalu1 = {};
	nalu1.size = sizeof(kH264Sps);
	nalu1.h264.type = H264_NALU_TYPE_SPS;
	ret = mbuf_coded_video_frame_add_nalu(frame, mem1, 0, &nalu1);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	mbuf_mem_unref(mem1);

	/* Slice IDR NALU */
	struct mbuf_mem *mem2 = nullptr;
	ret = mbuf_mem_generic_new(100, &mem2);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	struct vdef_nalu nalu2 = {};
	nalu2.size = 100;
	nalu2.h264.type = H264_NALU_TYPE_SLICE_IDR;
	ret = mbuf_coded_video_frame_add_nalu(frame, mem2, 0, &nalu2);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	mbuf_mem_unref(mem2);

	ret = mbuf_coded_video_frame_finalize(frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	ret = mbuf_coded_video_frame_queue_push(queue, frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	mbuf_coded_video_frame_unref(frame);

	/* Process queue and wait for delivery */
	bool delivered = loop.pumpUntil(
		[]() { return g_librtmp_mock.send_video_frame_call_count > 0; },
		2000);
	CU_ASSERT_TRUE(delivered);

	/* Close */
	closeRtmpStreamMuxer(muxer, loop, &muxerListener);
}

static void testRtmpStreamMuxerBackpressure()
{
	librtmp_mock_reset();
	g_librtmp_mock.trigger_connection_success_async = true;

	TestPompLoop loop;
	TestPdrawListener sessionListener;
	TestSession testSession(&loop, &sessionListener);
	IPdraw *session = testSession.get();

	std::string url = "rtmp://127.0.0.1:1935/live/stream";

	RtmpMuxerListener muxerListener;
	TrackingCodedVideoSourceListener srcListener;
	IPdraw::ICodedVideoSource *source = nullptr;

	IPdraw::IMuxer *muxer = openRtmpStreamMuxer(session,
						    loop,
						    url,
						    &muxerListener,
						    &source,
						    &srcListener,
						    &sessionListener);
	auto sourceOwner = std::unique_ptr<IPdraw::ICodedVideoSource>(source);

	/* Wait for connected callback */
	bool gotConnected = loop.pumpUntil(
		[&muxerListener]() { return muxerListener.mGotConnected; },
		5000);
	CU_ASSERT_TRUE(gotConnected);

	/* Test backpressure / dropped frame handling (EAGAIN) */
	g_librtmp_mock.send_should_fail_eagain = true;

	struct mbuf_coded_video_frame_queue *queue = source->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(queue);

	struct vdef_coded_frame frameInfo = {};
	frameInfo.format = vdef_h264_avcc;
	frameInfo.info.resolution.width = 1920;
	frameInfo.info.resolution.height = 800;
	frameInfo.info.bit_depth = 8;
	frameInfo.info.timescale = 1000000;
	frameInfo.info.timestamp = 0;
	frameInfo.type = VDEF_CODED_FRAME_TYPE_IDR;

	struct mbuf_coded_video_frame *frame = nullptr;
	int ret = mbuf_coded_video_frame_new(&frameInfo, &frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	struct mbuf_mem *mem = nullptr;
	ret = mbuf_mem_generic_new(100, &mem);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	struct vdef_nalu nalu = {};
	nalu.size = 100;
	nalu.h264.type = H264_NALU_TYPE_SLICE_IDR;
	ret = mbuf_coded_video_frame_add_nalu(frame, mem, 0, &nalu);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	mbuf_mem_unref(mem);

	ret = mbuf_coded_video_frame_finalize(frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	ret = mbuf_coded_video_frame_queue_push(queue, frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	mbuf_coded_video_frame_unref(frame);

	bool dropped = loop.pumpUntil(
		[muxer]() {
			struct pdraw_muxer_stats s = {};
			int r = muxer->getStats(&s);
			return (r == 0) && (s.rtmp.dropped_video_frames > 0);
		},
		2000);
	CU_ASSERT_TRUE(dropped);

	/* Cleanup mock status to avoid breaking reconnection/configure sequence
	 * during teardown */
	g_librtmp_mock.send_should_fail_eagain = false;

	/* Close */
	closeRtmpStreamMuxer(muxer, loop, &muxerListener);
}

static void testRtmpStreamMuxerFlush()
{
	librtmp_mock_reset();
	g_librtmp_mock.trigger_connection_success_async = true;

	TestPompLoop loop;
	TestPdrawListener sessionListener;
	TestSession testSession(&loop, &sessionListener);
	IPdraw *session = testSession.get();

	std::string url = "rtmp://127.0.0.1:1935/live/stream";

	RtmpMuxerListener muxerListener;
	TrackingCodedVideoSourceListener srcListener;
	IPdraw::ICodedVideoSource *source = nullptr;

	IPdraw::IMuxer *muxer = openRtmpStreamMuxer(session,
						    loop,
						    url,
						    &muxerListener,
						    &source,
						    &srcListener,
						    &sessionListener);
	auto sourceOwner = std::unique_ptr<IPdraw::ICodedVideoSource>(source);

	/* Wait for connected callback */
	bool gotConnected = loop.pumpUntil(
		[&muxerListener]() { return muxerListener.mGotConnected; },
		5000);
	CU_ASSERT_TRUE(gotConnected);

	/* Test onChannelFlush */
	int ret = source->flush();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotFlushed = loop.pumpUntil(
		[&srcListener]() { return srcListener.mGotFlushed; }, 2000);
	CU_ASSERT_TRUE(gotFlushed);

	/* Close */
	closeRtmpStreamMuxer(muxer, loop, &muxerListener);
}

static void testRtmpStreamMuxerDrain()
{
	librtmp_mock_reset();
	g_librtmp_mock.trigger_connection_success_async = true;

	TestPompLoop loop;
	TestPdrawListener sessionListener;
	TestSession testSession(&loop, &sessionListener);
	IPdraw *session = testSession.get();

	std::string url = "rtmp://127.0.0.1:1935/live/stream";

	RtmpMuxerListener muxerListener;
	TrackingCodedVideoSourceListener srcListener;
	IPdraw::ICodedVideoSource *source = nullptr;

	IPdraw::IMuxer *muxer = openRtmpStreamMuxer(session,
						    loop,
						    url,
						    &muxerListener,
						    &source,
						    &srcListener,
						    &sessionListener);
	auto sourceOwner = std::unique_ptr<IPdraw::ICodedVideoSource>(source);

	/* Wait for connected callback */
	bool gotConnected = loop.pumpUntil(
		[&muxerListener]() { return muxerListener.mGotConnected; },
		5000);
	CU_ASSERT_TRUE(gotConnected);

	/* Push one frame to put source in UNFLUSHED state so drain propagates
	 * down the channel */
	struct mbuf_coded_video_frame_queue *queue = source->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(queue);

	struct vdef_coded_frame frameInfo = {};
	frameInfo.format = vdef_h264_avcc;
	frameInfo.info.resolution.width = 1920;
	frameInfo.info.resolution.height = 800;
	frameInfo.info.bit_depth = 8;
	frameInfo.info.timescale = 1000000;
	frameInfo.info.timestamp = 0;
	frameInfo.type = VDEF_CODED_FRAME_TYPE_IDR;

	struct mbuf_coded_video_frame *frame = nullptr;
	int ret = mbuf_coded_video_frame_new(&frameInfo, &frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	struct mbuf_mem *mem = nullptr;
	ret = mbuf_mem_generic_new(100, &mem);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	struct vdef_nalu nalu = {};
	nalu.size = 100;
	nalu.h264.type = H264_NALU_TYPE_SLICE_IDR;
	ret = mbuf_coded_video_frame_add_nalu(frame, mem, 0, &nalu);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	mbuf_mem_unref(mem);

	ret = mbuf_coded_video_frame_finalize(frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	ret = mbuf_coded_video_frame_queue_push(queue, frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	mbuf_coded_video_frame_unref(frame);

	/* Test onChannelDrain */
	ret = source->drain();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotDrained = loop.pumpUntil(
		[&srcListener]() { return srcListener.mGotDrained; }, 5000);
	CU_ASSERT_TRUE(gotDrained);

	/* Close */
	closeRtmpStreamMuxer(muxer, loop, &muxerListener);
}

static void testRtmpStreamMuxerWatchdog()
{
	librtmp_mock_reset();
	g_librtmp_mock.trigger_connection_success_async = true;

	TestPompLoop loop;
	TestPdrawListener sessionListener;
	TestSession testSession(&loop, &sessionListener);
	IPdraw *session = testSession.get();

	std::string url = "rtmp://127.0.0.1:1935/live/stream";

	RtmpMuxerListener muxerListener;
	TrackingCodedVideoSourceListener srcListener;
	IPdraw::ICodedVideoSource *source = nullptr;

	IPdraw::IMuxer *muxer = openRtmpStreamMuxer(session,
						    loop,
						    url,
						    &muxerListener,
						    &source,
						    &srcListener,
						    &sessionListener);
	auto sourceOwner = std::unique_ptr<IPdraw::ICodedVideoSource>(source);

	/* Wait for connected callback */
	bool gotConnected = loop.pumpUntil(
		[&muxerListener]() { return muxerListener.mGotConnected; },
		5000);
	CU_ASSERT_TRUE(gotConnected);

	/* Cast to RtmpStreamMuxer to test internal watchdog */
	MuxerWrapper *wrapper = static_cast<MuxerWrapper *>(muxer);
	RtmpStreamMuxer *rtmpMuxer =
		static_cast<RtmpStreamMuxer *>(wrapper->getMuxer());
	CU_ASSERT_PTR_NOT_NULL_FATAL(rtmpMuxer);

	/* Test internal onConnectionWatchdog() */
	muxerListener.mGotConnected = false;
	rtmpMuxer->onConnectionWatchdog();
	bool reconnected = loop.pumpUntil(
		[&muxerListener]() { return muxerListener.mGotConnected; },
		5000);
	CU_ASSERT_TRUE(reconnected);

	/* Close */
	closeRtmpStreamMuxer(muxer, loop, &muxerListener);
}

static void testRtmpStreamMuxerConnectFail()
{
	librtmp_mock_reset();
	/* Make connection fail asynchronously during handshake/initial setup */
	g_librtmp_mock.trigger_connection_success_async = false;
	g_librtmp_mock.trigger_disconnection_async = true;
	g_librtmp_mock.disconnection_reason =
		RTMP_CLIENT_DISCONNECTION_REASON_REFUSED;

	TestPompLoop loop;
	TestPdrawListener sessionListener;
	TestSession testSession(&loop, &sessionListener);
	IPdraw *session = testSession.get();

	std::string url = "rtmp://127.0.0.1:1935/live/stream";

	RtmpMuxerListener muxerListener;
	TrackingCodedVideoSourceListener srcListener;
	IPdraw::ICodedVideoSource *source = nullptr;

	IPdraw::IMuxer *muxer = openRtmpStreamMuxer(session,
						    loop,
						    url,
						    &muxerListener,
						    &source,
						    &srcListener,
						    &sessionListener);
	auto sourceOwner = std::unique_ptr<IPdraw::ICodedVideoSource>(source);

	/* Wait for unrecoverable error callback */
	bool gotError = loop.pumpUntil(
		[&muxerListener]() {
			return muxerListener.mGotUnrecoverableError;
		},
		5000);
	CU_ASSERT_TRUE(gotError);

	/* Close */
	closeRtmpStreamMuxer(muxer, loop, &muxerListener);
}

static void testRtmpStreamMuxerReconnect()
{
	librtmp_mock_reset();
	g_librtmp_mock.trigger_connection_success_async = true;

	TestPompLoop loop;
	TestPdrawListener sessionListener;
	TestSession testSession(&loop, &sessionListener);
	IPdraw *session = testSession.get();

	std::string url = "rtmp://127.0.0.1:1935/live/stream";

	RtmpMuxerListener muxerListener;
	TrackingCodedVideoSourceListener srcListener;
	IPdraw::ICodedVideoSource *source = nullptr;

	IPdraw::IMuxer *muxer = openRtmpStreamMuxer(session,
						    loop,
						    url,
						    &muxerListener,
						    &source,
						    &srcListener,
						    &sessionListener);
	auto sourceOwner = std::unique_ptr<IPdraw::ICodedVideoSource>(source);

	/* Wait for connected callback */
	bool gotConnected = loop.pumpUntil(
		[&muxerListener]() { return muxerListener.mGotConnected; },
		5000);
	CU_ASSERT_TRUE(gotConnected);

	/* Trigger disconnection asynchronously using the mock trigger function
	 */
	muxerListener.mGotConnected = false;
	librtmp_mock_trigger_disconnection(
		RTMP_CLIENT_DISCONNECTION_REASON_NETWORK_ERROR);

	/* In pdraw_muxer_stream_rtmp.cpp, a timer of 2000ms is scheduled.
	 * We reset flags so that the next reconnection connect attempt will
	 * succeed */
	g_librtmp_mock.trigger_connection_success_async = true;

	/* Wait for the reconnection to trigger and connect again */
	bool reconnected = loop.pumpUntil(
		[&muxerListener]() { return muxerListener.mGotConnected; },
		6000);
	CU_ASSERT_TRUE(reconnected);

	/* Close */
	closeRtmpStreamMuxer(muxer, loop, &muxerListener);
}

static void testRtmpStreamMuxerReconnectRefused()
{
	librtmp_mock_reset();
	g_librtmp_mock.trigger_connection_success_async = true;

	TestPompLoop loop;
	TestPdrawListener sessionListener;
	TestSession testSession(&loop, &sessionListener);
	IPdraw *session = testSession.get();

	std::string url = "rtmp://127.0.0.1:1935/live/stream";

	RtmpMuxerListener muxerListener;
	TrackingCodedVideoSourceListener srcListener;
	IPdraw::ICodedVideoSource *source = nullptr;

	IPdraw::IMuxer *muxer = openRtmpStreamMuxer(session,
						    loop,
						    url,
						    &muxerListener,
						    &source,
						    &srcListener,
						    &sessionListener);
	auto sourceOwner = std::unique_ptr<IPdraw::ICodedVideoSource>(source);

	/* Wait for connected callback */
	bool gotConnected = loop.pumpUntil(
		[&muxerListener]() { return muxerListener.mGotConnected; },
		5000);
	CU_ASSERT_TRUE(gotConnected);

	/* Trigger disconnection with reason REFUSED.
	 * Since mHasBeenConnected is true, reconnection strategy says:
	 * doReconnect = true, reconnectionCount = 1. */
	muxerListener.mGotConnected = false;
	librtmp_mock_trigger_disconnection(
		RTMP_CLIENT_DISCONNECTION_REASON_REFUSED);

	/* Reset mock so reconnection succeeds */
	g_librtmp_mock.trigger_connection_success_async = true;

	/* Wait for reconnection to trigger and connect successfully */
	bool reconnected = loop.pumpUntil(
		[&muxerListener]() { return muxerListener.mGotConnected; },
		6000);
	CU_ASSERT_TRUE(reconnected);

	/* Close */
	closeRtmpStreamMuxer(muxer, loop, &muxerListener);
}

static void testRtmpStreamMuxerDisconnectClientRequest()
{
	librtmp_mock_reset();
	g_librtmp_mock.trigger_connection_success_async = true;

	TestPompLoop loop;
	TestPdrawListener sessionListener;
	TestSession testSession(&loop, &sessionListener);
	IPdraw *session = testSession.get();

	std::string url = "rtmp://127.0.0.1:1935/live/stream";

	RtmpMuxerListener muxerListener;
	TrackingCodedVideoSourceListener srcListener;
	IPdraw::ICodedVideoSource *source = nullptr;

	IPdraw::IMuxer *muxer = openRtmpStreamMuxer(session,
						    loop,
						    url,
						    &muxerListener,
						    &source,
						    &srcListener,
						    &sessionListener);
	auto sourceOwner = std::unique_ptr<IPdraw::ICodedVideoSource>(source);

	/* Wait for connected callback */
	bool gotConnected = loop.pumpUntil(
		[&muxerListener]() { return muxerListener.mGotConnected; },
		5000);
	CU_ASSERT_TRUE(gotConnected);

	/* Trigger disconnection with reason CLIENT_REQUEST. Reconnection
	 * strategy says: Do not reconnect, notify unrecoverable error. */
	muxerListener.mGotConnected = false;
	librtmp_mock_trigger_disconnection(
		RTMP_CLIENT_DISCONNECTION_REASON_CLIENT_REQUEST);

	/* Wait for unrecoverable error callback */
	bool gotError = loop.pumpUntil(
		[&muxerListener]() {
			return muxerListener.mGotUnrecoverableError;
		},
		5000);
	CU_ASSERT_TRUE(gotError);

	/* Close */
	closeRtmpStreamMuxer(muxer, loop, &muxerListener);
}


/* testCxxStreamMuxerServerPushedAnnounce */

/* Exercises RtmpStreamMuxer::onChannelFlush (l. 616) with an active RTMP
 * client (mRtmpClient != nullptr) so that rtmp_client_flush() is reached. */
static void testRtmpStreamMuxerFlushAfterConnect()
{
	librtmp_mock_reset();
	g_librtmp_mock.trigger_connection_success_async = true;

	TestPompLoop loop;
	TestPdrawListener sessionListener;
	TestSession testSession(&loop, &sessionListener);
	IPdraw *session = testSession.get();

	std::string url = "rtmp://127.0.0.1:1935/live/stream";

	RtmpMuxerListener muxerListener;
	TrackingCodedVideoSourceListener srcListener;
	IPdraw::ICodedVideoSource *source = nullptr;

	IPdraw::IMuxer *muxer = openRtmpStreamMuxer(session,
						    loop,
						    url,
						    &muxerListener,
						    &source,
						    &srcListener,
						    &sessionListener);
	auto sourceOwner = std::unique_ptr<IPdraw::ICodedVideoSource>(source);

	/* Wait for connected callback — ensures mRtmpClient is set */
	bool gotConnected = loop.pumpUntil(
		[&muxerListener]() { return muxerListener.mGotConnected; },
		5000);
	CU_ASSERT_TRUE(gotConnected);

	/* Push one frame to put the source in UNFLUSHED state so that
	 * source->flush() actually propagates onChannelFlush to the muxer */
	struct mbuf_coded_video_frame_queue *queue = source->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(queue);

	struct vdef_coded_frame frameInfo = {};
	frameInfo.format = vdef_h264_avcc;
	frameInfo.info.resolution.width = 1920;
	frameInfo.info.resolution.height = 800;
	frameInfo.info.bit_depth = 8;
	frameInfo.info.timescale = 1000000;
	frameInfo.info.timestamp = 0;
	frameInfo.type = VDEF_CODED_FRAME_TYPE_IDR;

	struct mbuf_coded_video_frame *frame = nullptr;
	int ret = mbuf_coded_video_frame_new(&frameInfo, &frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	struct mbuf_mem *mem = nullptr;
	ret = mbuf_mem_generic_new(100, &mem);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	struct vdef_nalu nalu = {};
	nalu.size = 100;
	nalu.h264.type = H264_NALU_TYPE_SLICE_IDR;
	ret = mbuf_coded_video_frame_add_nalu(frame, mem, 0, &nalu);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	mbuf_mem_unref(mem);

	ret = mbuf_coded_video_frame_finalize(frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	ret = mbuf_coded_video_frame_queue_push(queue, frame);
	CU_ASSERT_EQUAL(ret, 0);
	mbuf_coded_video_frame_unref(frame);

	/* Flush AFTER connection: onChannelFlush branches into
	 * rtmp_client_flush */
	int prevFlush = g_librtmp_mock.flush_call_count;
	ret = source->flush();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotFlushed = loop.pumpUntil(
		[&srcListener]() { return srcListener.mGotFlushed; }, 2000);
	CU_ASSERT_TRUE(gotFlushed);
	CU_ASSERT_EQUAL(g_librtmp_mock.flush_call_count, prevFlush + 1);

	closeRtmpStreamMuxer(muxer, loop, &muxerListener);
}

/* Exercises RtmpStreamMuxer::onSocketCreated (l. 727): the mock now fires
 * socket_cb with fd=-1 just before CONNECTED, so socketCreated() is called
 * on the session and socket_cb_call_count is incremented. */
static void testRtmpStreamMuxerSocketCreated()
{
	librtmp_mock_reset();
	g_librtmp_mock.trigger_connection_success_async = true;

	TestPompLoop loop;
	TestPdrawListener sessionListener;
	TestSession testSession(&loop, &sessionListener);
	IPdraw *session = testSession.get();

	std::string url = "rtmp://127.0.0.1:1935/live/stream";

	RtmpMuxerListener muxerListener;
	TrackingCodedVideoSourceListener srcListener;
	IPdraw::ICodedVideoSource *source = nullptr;

	IPdraw::IMuxer *muxer = openRtmpStreamMuxer(session,
						    loop,
						    url,
						    &muxerListener,
						    &source,
						    &srcListener,
						    nullptr);
	auto sourceOwner = std::unique_ptr<IPdraw::ICodedVideoSource>(source);

	/* Wait for connected callback — socket_cb is fired just before it */
	bool gotConnected = loop.pumpUntil(
		[&muxerListener]() { return muxerListener.mGotConnected; },
		5000);
	CU_ASSERT_TRUE(gotConnected);

	/* Verify onSocketCreated was invoked via socket_cb */
	CU_ASSERT_EQUAL(g_librtmp_mock.socket_cb_call_count, 1);

	closeRtmpStreamMuxer(muxer, loop, &muxerListener);
}

/* Exercises the "only 1 video media supported" rejection path in
 * RtmpStreamMuxer::addInputMedia() (pdraw_muxer_stream_rtmp.cpp l. 184):
 * once a first coded video media has been added successfully, adding a
 * second one must be rejected with -EALREADY, and the first video media
 * must remain the one in use (no side effect on mVideoMedia). */
static void testRtmpStreamMuxerAddSecondVideoMediaRejected()
{
	librtmp_mock_reset();
	g_librtmp_mock.trigger_connection_success_async = true;

	TestPompLoop loop;
	TestPdrawListener sessionListener;
	TestSession testSession(&loop, &sessionListener);
	IPdraw *session = testSession.get();

	std::string url = "rtmp://127.0.0.1:1935/live/stream";

	RtmpMuxerListener muxerListener;
	TrackingCodedVideoSourceListener srcListener;
	IPdraw::ICodedVideoSource *source = nullptr;

	/* openRtmpStreamMuxer() creates the muxer and a first coded video
	 * source, waits for its media to be added, and calls addMedia() on
	 * it (the first video media -- accepted). */
	IPdraw::IMuxer *muxer = openRtmpStreamMuxer(session,
						    loop,
						    url,
						    &muxerListener,
						    &source,
						    &srcListener,
						    &sessionListener);
	auto sourceOwner = std::unique_ptr<IPdraw::ICodedVideoSource>(source);

	/* Wait for connected callback */
	bool gotConnected = loop.pumpUntil(
		[&muxerListener]() { return muxerListener.mGotConnected; },
		5000);
	CU_ASSERT_TRUE(gotConnected);

	/* Create a second coded video source and get its media added */
	struct pdraw_video_source_params srcParams2 = {};
	srcParams2.video.format = VDEF_FRAME_TYPE_CODED;
	srcParams2.video.coded.format = vdef_h264_avcc;
	memcpy(srcParams2.video.coded.h264.sps, kH264Sps, sizeof(kH264Sps));
	srcParams2.video.coded.h264.spslen = sizeof(kH264Sps);
	memcpy(srcParams2.video.coded.h264.pps, kH264Pps, sizeof(kH264Pps));
	srcParams2.video.coded.h264.ppslen = sizeof(kH264Pps);

	TrackingCodedVideoSourceListener srcListener2;
	IPdraw::ICodedVideoSource *source2 = nullptr;
	int ret = session->createCodedVideoSource(
		&srcParams2, &srcListener2, &source2);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source2);
	auto source2Owner = std::unique_ptr<IPdraw::ICodedVideoSource>(source2);

	bool gotSecondMedia = loop.pumpUntil(
		[&sessionListener]() {
			return sessionListener.findCodedVideoMedia(1) !=
			       nullptr;
		},
		1000);
	CU_ASSERT_TRUE_FATAL(gotSecondMedia);
	unsigned int secondMediaId = sessionListener.findCodedVideoMedia(1)->id;

	ret = muxer->addMedia(secondMediaId, nullptr);
	CU_ASSERT_EQUAL(ret, -EALREADY);

	source2Owner.reset();
	sourceOwner.reset();

	/* Close */
	closeRtmpStreamMuxer(muxer, loop, &muxerListener);
}

/* Exercises the "unsupported input media" rejection path in
 * RtmpStreamMuxer::addInputMedia() (pdraw_muxer_stream_rtmp.cpp l. 176-181):
 * a media that is not a CodedVideoMedia (here a raw video media) fails the
 * dynamic_cast<CodedVideoMedia *> and must be rejected with -ENOSYS. */
static void testRtmpStreamMuxerAddNonCodedMediaRejected()
{
	librtmp_mock_reset();
	g_librtmp_mock.trigger_connection_success_async = true;

	TestPompLoop loop;
	TestPdrawListener sessionListener;
	TestSession testSession(&loop, &sessionListener);
	IPdraw *session = testSession.get();

	std::string url = "rtmp://127.0.0.1:1935/live/stream";

	struct pdraw_muxer_params muxerParams = {};
	RtmpMuxerListener muxerListener;
	IPdraw::IMuxer *muxer = nullptr;
	int ret =
		session->createMuxer(url, &muxerParams, &muxerListener, &muxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(muxer);

	/* Create a raw video source: its media is not a CodedVideoMedia */
	struct pdraw_video_source_params rawSrcParams = {};
	rawSrcParams.video.format = VDEF_FRAME_TYPE_RAW;

	StubRawVideoSourceListener rawSrcListener;
	IPdraw::IRawVideoSource *rawSource = nullptr;
	ret = session->createRawVideoSource(
		&rawSrcParams, &rawSrcListener, &rawSource);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(rawSource);
	auto rawSourceOwner =
		std::unique_ptr<IPdraw::IRawVideoSource>(rawSource);

	bool gotRawMedia = loop.pumpUntil(
		[&sessionListener]() {
			return sessionListener.findRawVideoMedia() != nullptr;
		},
		1000);
	CU_ASSERT_TRUE_FATAL(gotRawMedia);
	unsigned int rawMediaId = sessionListener.findRawVideoMedia()->id;

	ret = muxer->addMedia(rawMediaId, nullptr);
	CU_ASSERT_EQUAL(ret, -ENOSYS);

	rawSourceOwner.reset();

	/* Wait for connected callback before closing, as done in every other
	 * test in this file */
	bool gotConnected = loop.pumpUntil(
		[&muxerListener]() { return muxerListener.mGotConnected; },
		5000);
	CU_ASSERT_TRUE(gotConnected);

	/* Close */
	closeRtmpStreamMuxer(muxer, loop, &muxerListener);
}


static void testRtmpStreamMuxerAddInputMediaNull()
{
	librtmp_mock_reset();

	TestPompLoop loop;
	TestPdrawListener sessionListener;
	TestSession testSession(&loop, &sessionListener);
	IPdraw *session = testSession.get();

	std::string url = "rtmp://127.0.0.1:1935/live/stream";

	struct pdraw_muxer_params muxerParams = {};
	RtmpMuxerListener muxerListener;
	IPdraw::IMuxer *muxer = nullptr;
	int ret =
		session->createMuxer(url, &muxerParams, &muxerListener, &muxer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(muxer);

	auto *wrapper = static_cast<MuxerWrapper *>(muxer);
	auto *rtmpMuxer = static_cast<RtmpStreamMuxer *>(wrapper->getMuxer());
	CU_ASSERT_PTR_NOT_NULL_FATAL(rtmpMuxer);

	/* Call addInputMedia with null media */
	ret = rtmpMuxer->addInputMedia(nullptr);
	CU_ASSERT_EQUAL(ret, -ENOSYS);

	/* Close */
	closeRtmpStreamMuxer(muxer, loop, &muxerListener);
}

CU_TestInfo g_pdraw_test_pipeline_muxer_stream_rtmp[] = {
	{(char *)"testCxxRtmpStreamMuxerSuccess", testRtmpStreamMuxerSuccess},
	{(char *)"testCxxRtmpStreamMuxerUnpacked", testRtmpStreamMuxerUnpacked},
	{(char *)"testCxxRtmpStreamMuxerBackpressure",
	 testRtmpStreamMuxerBackpressure},
	{(char *)"testCxxRtmpStreamMuxerFlush", testRtmpStreamMuxerFlush},
	{(char *)"testCxxRtmpStreamMuxerDrain", testRtmpStreamMuxerDrain},
	{(char *)"testCxxRtmpStreamMuxerWatchdog", testRtmpStreamMuxerWatchdog},
	{(char *)"testCxxRtmpStreamMuxerConnectFail",
	 testRtmpStreamMuxerConnectFail},
	{(char *)"testCxxRtmpStreamMuxerReconnect",
	 testRtmpStreamMuxerReconnect},
	{(char *)"testCxxRtmpStreamMuxerReconnectRefused",
	 testRtmpStreamMuxerReconnectRefused},
	{(char *)"testCxxRtmpStreamMuxerDisconnectClientRequest",
	 testRtmpStreamMuxerDisconnectClientRequest},
	{(char *)"testCxxRtmpStreamMuxerFlushAfterConnect",
	 testRtmpStreamMuxerFlushAfterConnect},
	{(char *)"testCxxRtmpStreamMuxerSocketCreated",
	 testRtmpStreamMuxerSocketCreated},
	{(char *)"testCxxRtmpStreamMuxerAddSecondVideoMediaRejected",
	 testRtmpStreamMuxerAddSecondVideoMediaRejected},
	{(char *)"testCxxRtmpStreamMuxerAddNonCodedMediaRejected",
	 testRtmpStreamMuxerAddNonCodedMediaRejected},
	{(char *)"testCxxRtmpStreamMuxerAddInputMediaNull",
	 testRtmpStreamMuxerAddInputMediaNull},
	CU_TEST_INFO_NULL,
};
