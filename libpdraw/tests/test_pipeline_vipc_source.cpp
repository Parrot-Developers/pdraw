/**
 * Parrot Drones Audio and Video Vector library
 * libpdraw unit tests — Video IPC source against a real, self-contained
 * SHM-backed server (Tier B, self-contained fixture)
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

/* Requires CONFIG_PDRAW_VIPC_BACKEND_SHM=y.
 *
 * libpdraw-backend/tests/pdraw_vipcsourcesink_test.c is the reference
 * client program for a VipcSource, but it expects an already-running
 * *external* server (its VIPC address is a CLI argument) -- there is no
 * self-contained loopback anywhere in the monorepo, and no SHM-backed
 * server at all: libvideo-ipc/tools/ipc-server.c only implements the HISI
 * and DMABUF backends. VipcTestServer below is a from-scratch minimal one,
 * built directly against libvideo-ipc's server API, giving a genuine
 * end-to-end test: a real SHM-backed Video IPC server process (in the same
 * pomp_loop, for simplicity) feeding a real VipcSource, no external
 * process, no camera hardware, no kernel module (the SHM backend is pure
 * userspace).
 *
 * Unlike the other test_api_* files, this suite does NOT use the shared
 * g_test_session (its IPdraw::Listener is fixed to nullptr at construction,
 * see test_fixtures.hpp). Observing onMediaAdded() for the VIPC source's
 * output media requires a real session-wide listener, so the test below
 * builds its own private TestPompLoop + TestSession, fully self-contained.
 * No suite init/cleanup is registered for this file (see test_main.c: NULL,
 * NULL). */

#ifdef BUILD_LIBVIDEO_IPC
#	define PDRAW_TEST_VIPC_SOURCE_ENABLED 1
#endif

#define ULOG_TAG pdraw_test_pipeline_vipc
#include "test_api_common.hpp"
#include "test_common.h"
#include "test_fixtures.hpp"

#include <media-buffers/mbuf_mem.h>
#include <media-buffers/mbuf_raw_video_frame.h>

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <string.h>

#include <atomic>
#include <string>

ULOG_DECLARE_TAG(ULOG_TAG);

#ifdef PDRAW_TEST_VIPC_SOURCE_ENABLED

#	include "pdraw_vipc_source.hpp"

#	include <video-acquisition/vacq.h>
#	include <video-ipc/vipc_server.h>
#	include <vipc_backend_shm/vipc_backend_shm.h>

using namespace Pdraw;
using namespace PdrawTest;


/* Anonymous namespace: MediaTrackingListener/VipcSourceTrackingListener/
 * VipcTestServer are duplicated (with different bodies) by name across
 * several test_pipeline_*.cpp files in this suite, all linked into the
 * same tst-libpdraw binary (see atom.mk: all-cpp-files-under,tests). A
 * plain `class Foo {...}` has EXTERNAL linkage by default in C++ -- with
 * the same class name defined differently in multiple translation units,
 * that is a real ODR violation, not just a style nit: the linker is free
 * to resolve any one TU's virtual calls (e.g. onMediaAdded()) to a
 * *different* TU's definition, whose member offsets/struct layout don't
 * match the object actually constructed here, silently corrupting fields
 * (confirmed the hard way: rawVideo->videoInfo.resolution.width read back
 * as 0 while .height read back correctly -- a debug print placed directly
 * inside *this* file's onMediaAdded() never even fired, proving a
 * different TU's copy ran instead). The anonymous namespace below gives
 * these classes real internal linkage, matching what "private to this
 * translation unit" was always meant to guarantee. */
namespace {

/* Session-wide listener recording the VIPC source's own (lazily created)
 * raw output media -- same rationale/pattern as MediaTrackingListener in
 * the other pipeline_* files (duplicated here rather than shared, per this
 * suite's convention: each test file's listeners are private to its own
 * translation unit). */
class MediaTrackingListener : public IPdraw::Listener {
public:
	struct Added {
		unsigned int id;
		enum vdef_frame_type videoFormat;
		struct vdef_format_info videoInfo;
	};

	void stopResponse(IPdraw * /*p*/, int status) override
	{
		mStopStatus = status;
		mGotStopResponse = true;
	}

	void onMediaAdded(IPdraw * /*p*/,
			  const struct pdraw_media_info *info,
			  void * /*elementUserData*/) override
	{
		if (info->type != PDRAW_MEDIA_TYPE_VIDEO)
			return;
		Added a = {};
		a.id = info->id;
		a.videoFormat = info->video.format;
		if (a.videoFormat == VDEF_FRAME_TYPE_RAW)
			a.videoInfo = info->video.raw.info;
		mAdded.push_back(a);
	}

	void onMediaRemoved(IPdraw * /*p*/,
			    const struct pdraw_media_info * /*i*/,
			    void * /*u*/) override
	{
	}

	void onSocketCreated(IPdraw * /*p*/, int /*fd*/) override {}

	const Added *findRawVideoMedia() const
	{
		for (const auto &a : mAdded) {
			if (a.videoFormat == VDEF_FRAME_TYPE_RAW)
				return &a;
		}
		return nullptr;
	}

	std::vector<Added> mAdded;
	bool mGotStopResponse = false;
	int mStopStatus = 0;
};


/* Tracks IVipcSource's own lifecycle events. */
class VipcSourceTrackingListener : public IPdraw::IVipcSource::Listener {
public:
	void
	vipcSourceReadyToPlay(IPdraw * /*p*/,
			      IPdraw::IVipcSource *src,
			      bool ready,
			      enum pdraw_vipc_source_eos_reason reason) override
	{
		mReady = ready;
		mGotReadyToPlay = true;
		mReadyToPlayCallCount++;
		mLastReadyToPlayReason = reason;
		/* testCxxVipcSourceCreateMediaRestartsRunningAfterIgnoredEos
		 * flips this to reentrantly call play() from inside this very
		 * callback -- mirroring a real app auto-resuming playback as
		 * soon as it is told ready -- which is what reaches
		 * VipcSource::createMedia()'s "if (mWasRunning && mRunning)"
		 * (l. 1468-1481): both flags true at once. */
		if (ready && mAutoPlayOnReady)
			(void)src->play();
	}

	void vipcSourcePlayResponse(IPdraw * /*p*/,
				    IPdraw::IVipcSource * /*src*/) override
	{
		mGotPlayResponse = true;
	}

	void vipcSourcePauseResponse(IPdraw * /*p*/,
				     IPdraw::IVipcSource * /*src*/) override
	{
		mGotPauseResponse = true;
	}

	bool vipcSourceFramerateChanged(
		IPdraw * /*p*/,
		IPdraw::IVipcSource * /*src*/,
		const struct vdef_frac *prevFramerate,
		const struct vdef_frac *newFramerate) override
	{
		mGotFramerateChanged = true;
		mFramerateChangedCallCount++;
		if (prevFramerate != nullptr)
			mPrevFramerate = *prevFramerate;
		if (newFramerate != nullptr)
			mNewFramerate = *newFramerate;
		/* Default false: recreate the media, the only path with an
		 * externally observable effect (a new media, at the new
		 * framerate). testCxxVipcSourceFramerateChangeIgnoredKeepsMedia
		 * flips this to exercise the opposite (ignore) branch of
		 * VipcSource::statusCb(). */
		return mFramerateChangeIgnoreResult;
	}

	void vipcSourceConfigured(IPdraw * /*p*/,
				  IPdraw::IVipcSource * /*src*/,
				  int /*status*/,
				  const struct vdef_format_info * /*info*/,
				  const struct vdef_rectf * /*crop*/) override
	{
	}

	void
	vipcSourceFrameReady(IPdraw * /*p*/,
			     IPdraw::IVipcSource * /*src*/,
			     struct mbuf_raw_video_frame * /*frame*/) override
	{
		mFrameReadyCount++;
	}

	bool vipcSourceEndOfStream(
		IPdraw * /*p*/,
		IPdraw::IVipcSource * /*src*/,
		enum pdraw_vipc_source_eos_reason eosReason) override
	{
		mGotEos = true;
		mLastEosReason = eosReason;
		/* Default false: destroy the media, the path covered by
		 * testCxxVipcSourceServerEosNotifiesListener.
		 * testCxxVipcSourceEosIgnoredNotifiesNotReady flips this to
		 * exercise the opposite (ignore/keep media) branch of
		 * VipcSource::eosCb(). */
		return mEosIgnoreResult;
	}

	bool mGotReadyToPlay = false;
	bool mReady = false;
	int mReadyToPlayCallCount = 0;
	enum pdraw_vipc_source_eos_reason mLastReadyToPlayReason =
		PDRAW_VIPC_SOURCE_EOS_REASON_NONE;
	bool mGotPlayResponse = false;
	bool mGotPauseResponse = false;
	std::atomic<int> mFrameReadyCount{0};
	bool mGotEos = false;
	enum pdraw_vipc_source_eos_reason mLastEosReason =
		PDRAW_VIPC_SOURCE_EOS_REASON_NONE;
	bool mGotFramerateChanged = false;
	int mFramerateChangedCallCount = 0;
	struct vdef_frac mPrevFramerate = {};
	struct vdef_frac mNewFramerate = {};
	bool mFramerateChangeIgnoreResult = false;
	bool mEosIgnoreResult = false;
	bool mAutoPlayOnReady = false;
};


/* Unlike g_stub_raw_video_sink_listener (a no-op stub used elsewhere), this
 * actually acknowledges flush/drain requests: required for
 * IVipcSource::pause()'s internal drain() to ever complete, and tracks
 * session metadata updates for the setSessionMetadata() roundtrip test. */
class QueueDrainingRawVideoSinkListener
		: public IPdraw::IRawVideoSink::Listener {
public:
	void
	onRawVideoSinkMediaAdded(IPdraw * /*p*/,
				 IPdraw::IRawVideoSink * /*sk*/,
				 const struct pdraw_media_info * /*i*/) override
	{
	}

	void onRawVideoSinkMediaRemoved(IPdraw * /*p*/,
					IPdraw::IRawVideoSink * /*sk*/,
					const struct pdraw_media_info * /*i*/,
					bool /*restart*/) override
	{
	}

	void onRawVideoSinkFlush(IPdraw * /*p*/,
				 IPdraw::IRawVideoSink *sk) override
	{
		struct mbuf_raw_video_frame *f = nullptr;
		while (mQueue != nullptr &&
		       mbuf_raw_video_frame_queue_pop(mQueue, &f) == 0)
			mbuf_raw_video_frame_unref(f);
		mFlushCount++;
		sk->queueFlushed();
	}

	void onRawVideoSinkDrain(IPdraw * /*p*/,
				 IPdraw::IRawVideoSink *sk) override
	{
		struct mbuf_raw_video_frame *f = nullptr;
		while (mQueue != nullptr &&
		       mbuf_raw_video_frame_queue_pop(mQueue, &f) == 0)
			mDrainedFrames.push_back(f);
		mDrainCount++;
		sk->queueDrained();
	}

	void onRawVideoSinkSessionMetaUpdate(
		IPdraw * /*p*/,
		IPdraw::IRawVideoSink * /*sk*/,
		const struct vmeta_session *meta) override
	{
		if (meta != nullptr)
			mLastSessionMeta = *meta;
		mGotSessionMetaUpdate = true;
	}

	/* Must be set right after createRawVideoSink() returns, before any
	 * flush/drain can occur. */
	struct mbuf_raw_video_frame_queue *mQueue = nullptr;

	/* Populated by onRawVideoSinkDrain(); caller must unref each entry. */
	std::vector<struct mbuf_raw_video_frame *> mDrainedFrames;

	/* Counts round-trips through VipcSource::onChannelFlushed() /
	 * onChannelDrained(): those are only reached once this listener
	 * acknowledges the request via queueFlushed()/queueDrained() above,
	 * which is what lets VipcSource::completeFlush() (and, from there,
	 * completeStop()/pauseResponse()) ever proceed. */
	int mFlushCount = 0;
	int mDrainCount = 0;

	bool mGotSessionMetaUpdate = false;
	struct vmeta_session mLastSessionMeta = {};
};


/* ── Minimal self-contained SHM Video IPC server ───────────────────────────
 * Key mechanics confirmed by reading libvideo-ipc/src/backend/shm/src/
 * shm-backend.c and src/server.c before writing this:
 *  - vipc_be_shm_init() does NOT create the SHM region itself on the server
 *    side (vipc_be_new() early-returns for server=true) -- the *caller*
 *    (this class) must create/mmap the backing file itself, at the exact
 *    path the client will independently derive and open: since the path
 *    passed here does not start with '/', the client
 *    (vipc_be_shm_client_init()) prepends "/dev/shm/" to it.
 *  - The "buffer" argument to vipcs_send_frame() is NOT frame pixel data:
 *    for the SHM backend it is interpreted as a pointer to the frame's
 *    intended SHM slot index (vipc_be_shm_store_buffer() memcpy()s
 *    sizeof(vipc_frame::index) bytes from it into frame->index) -- the
 *    actual pixel bytes must already be written into that slot's offset in
 *    the mmap'd region *before* calling vipcs_send_frame().
 *  - vipcs_send_frame() only delivers to remotes with remote->started ==
 *    true (server.c): a client that has connected but not yet called
 *    play() (-> vipcc_start()) is skipped entirely, and if no remote
 *    received it the frame is immediately released instead of queued --
 *    hence waiting on runningRemoteCount() below before ever sending,
 *    to avoid sending before the client's play() request has actually
 *    been processed server-side (play() returning 0 only means the
 *    *client* dispatched the request, not that the server has seen it
 *    yet). */
class VipcTestServer {
public:
	static constexpr uint32_t kWidth = 64;
	static constexpr uint32_t kHeight = 64;
	static constexpr size_t kYSize = kWidth * kHeight;
	static constexpr size_t kUvSize = kWidth * (kHeight / 2);
	static constexpr size_t kEntrySize = kYSize + kUvSize;
	static constexpr uint32_t kNumEntries = 4;

	VipcTestServer(struct pomp_loop *loop, const char *pompAddr)
	{
		/* Relative path (no leading '/'): the client prepends
		 * "/dev/shm/" itself (see shm-backend.c:
		 * vipc_be_shm_client_init()/set_vipc_path()). */
		char shmName[64];
		snprintf(shmName,
			 sizeof(shmName),
			 "pdraw_test_vipc_%p",
			 static_cast<void *>(this));
		mRelPath = shmName;
		mAbsPath = std::string("/dev/shm/") + mRelPath;

		mShmFd =
			open(mAbsPath.c_str(), O_CREAT | O_RDWR | O_EXCL, 0600);
		CU_ASSERT_FATAL(mShmFd >= 0);

		int ret = ftruncate(mShmFd, kEntrySize * kNumEntries);
		CU_ASSERT_EQUAL_FATAL(ret, 0);

		void *addr = mmap(nullptr,
				  kEntrySize * kNumEntries,
				  PROT_READ | PROT_WRITE,
				  MAP_SHARED,
				  mShmFd,
				  0);
		CU_ASSERT_FATAL(addr != MAP_FAILED);
		mShmAddr = static_cast<uint8_t *>(addr);

		mBeCtx = vipc_be_shm_init(
			mRelPath.c_str(), kEntrySize, kNumEntries);
		CU_ASSERT_PTR_NOT_NULL_FATAL(mBeCtx);

		struct vipcs_cb serverCbs = {};
		serverCbs.frame_release_cb = &VipcTestServer::frameReleaseCb;
		serverCbs.running_remotes_cb =
			&VipcTestServer::runningRemotesCb;

		mCtx = vipcs_new(loop,
				 &serverCbs,
				 mBeCtx,
				 &vipc_be_shm_callbacks,
				 pompAddr,
				 kNumEntries,
				 kNumEntries,
				 this);
		CU_ASSERT_PTR_NOT_NULL_FATAL(mCtx);

		struct vipc_status status = makeStatus(30, 1);
		ret = vipcs_send_status(mCtx, &status);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
	}

	~VipcTestServer()
	{
		if (mCtx != nullptr)
			vipcs_destroy(mCtx);
		if (mBeCtx != nullptr)
			vipc_be_shm_destroy(mBeCtx);
		if (mShmAddr != nullptr)
			munmap(mShmAddr, kEntrySize * kNumEntries);
		if (mShmFd >= 0)
			close(mShmFd);
		if (!mAbsPath.empty())
			unlink(mAbsPath.c_str());
	}

	int runningRemoteCount() const
	{
		return mRunningRemotes.load();
	}

	/* Re-sends the status with the given framerate (same resolution and
	 * format otherwise). Passing the same framerate as the last call (or
	 * as the constructor's initial send, 30/1) makes the resent status
	 * compare byte-for-byte identical to VipcSource's stored copy,
	 * reaching statusCb()'s "ignored (identical)" path; a different
	 * framerate instead reaches the "differs only by framerate" path,
	 * which calls the listener's vipcSourceFramerateChanged() before
	 * deciding whether to keep or recreate the media. */
	void sendStatusWithFramerate(uint32_t framerateNum,
				     uint32_t framerateDen)
	{
		struct vipc_status status =
			makeStatus(framerateNum, framerateDen);
		int ret = vipcs_send_status(mCtx, &status);
		CU_ASSERT_EQUAL(ret, 0);
	}

	void sendStatusWithFormat(enum vacq_pix_format format)
	{
		struct vipc_status status = makeStatus(30, 1, format);
		int ret = vipcs_send_status(mCtx, &status);
		CU_ASSERT_EQUAL(ret, 0);
	}

	/* Writes one flat-grey NV12 frame into the next SHM slot and sends
	 * it. Must only be called once runningRemoteCount() > 0 (see the
	 * class comment above for why). */
	void sendFrame(uint64_t tsNs)
	{
		uint32_t slot = mNextSlot++ % kNumEntries;
		uint8_t *dst = mShmAddr + kEntrySize * slot;
		memset(dst, 0x80, kYSize);
		memset(dst + kYSize, 0x80, kUvSize);

		auto *frame = static_cast<struct vipc_frame *>(
			calloc(1, sizeof(struct vipc_frame)));
		CU_ASSERT_PTR_NOT_NULL_FATAL(frame);
		frame->format = VACQ_PIX_FORMAT_NV12;
		frame->ts_sof_ns = tsNs;
		frame->ts_eof_ns = tsNs;
		frame->width = kWidth;
		frame->height = kHeight;
		frame->num_planes = 2;
		frame->planes[0].stride = kWidth;
		frame->planes[0].size = kYSize;
		frame->planes[1].stride = kWidth;
		frame->planes[1].size = kUvSize;
		frame->crop.left = 0.f;
		frame->crop.top = 0.f;
		frame->crop.width = 1.f;
		frame->crop.height = 1.f;

		int ret = vipcs_send_frame(mCtx, frame, &slot, this);
		CU_ASSERT_EQUAL(ret, 0);
	}

	/* Same as sendFrame(), but tagged with a different pixel format
	 * (still NV12-shaped: same plane count/strides/sizes, so the SHM
	 * slot layout stays valid) -- reaches VipcSource::processFrame()'s
	 * "invalid frame format" rejection (the media's format, established
	 * from the status this class sends at construction, is NV12; NV21 is
	 * a real, distinct vdef_raw_format -- see
	 * vacq_pix_format_to_vdef_raw_format() -- not just an unmapped
	 * value, so the frame is rejected by the format *comparison*, not by
	 * the format *conversion* failing first). */
	void sendFrameWithFormat(enum vacq_pix_format format, uint64_t tsNs)
	{
		uint32_t slot = mNextSlot++ % kNumEntries;
		uint8_t *dst = mShmAddr + kEntrySize * slot;
		memset(dst, 0x80, kYSize);
		memset(dst + kYSize, 0x80, kUvSize);

		auto *frame = static_cast<struct vipc_frame *>(
			calloc(1, sizeof(struct vipc_frame)));
		CU_ASSERT_PTR_NOT_NULL_FATAL(frame);
		frame->format = format;
		frame->ts_sof_ns = tsNs;
		frame->ts_eof_ns = tsNs;
		frame->width = kWidth;
		frame->height = kHeight;
		frame->num_planes = 2;
		frame->planes[0].stride = kWidth;
		frame->planes[0].size = kYSize;
		frame->planes[1].stride = kWidth;
		frame->planes[1].size = kUvSize;
		frame->crop.left = 0.f;
		frame->crop.top = 0.f;
		frame->crop.width = 1.f;
		frame->crop.height = 1.f;

		int ret = vipcs_send_frame(mCtx, frame, &slot, this);
		CU_ASSERT_EQUAL(ret, 0);
	}

	void sendFrameWithFullRange(uint32_t fullRange, uint64_t tsNs)
	{
		uint32_t slot = mNextSlot++ % kNumEntries;
		uint8_t *dst = mShmAddr + kEntrySize * slot;
		memset(dst, 0x80, kYSize);
		memset(dst + kYSize, 0x80, kUvSize);

		auto *frame = static_cast<struct vipc_frame *>(
			calloc(1, sizeof(struct vipc_frame)));
		CU_ASSERT_PTR_NOT_NULL_FATAL(frame);
		frame->format = VACQ_PIX_FORMAT_NV12;
		if (fullRange)
			frame->meta.flags |= VIPC_META_FLAG_FULL_RANGE;
		frame->ts_sof_ns = tsNs;
		frame->ts_eof_ns = tsNs;
		frame->width = kWidth;
		frame->height = kHeight;
		frame->num_planes = 2;
		frame->planes[0].stride = kWidth;
		frame->planes[0].size = kYSize;
		frame->planes[1].stride = kWidth;
		frame->planes[1].size = kUvSize;
		frame->crop.left = 0.f;
		frame->crop.top = 0.f;
		frame->crop.width = 1.f;
		frame->crop.height = 1.f;

		int ret = vipcs_send_frame(mCtx, frame, &slot, this);
		CU_ASSERT_EQUAL(ret, 0);
	}

	/* Same as sendFrame(), but at a different (smaller) NV12 resolution
	 * than kWidth/kHeight -- reaches VipcSource::processFrame()'s
	 * "invalid frame resolution" rejection (the media's resolution,
	 * established at kWidth/kHeight from the status, no longer matches).
	 * width/height must stay small enough for the resulting NV12 frame
	 * to still fit within the fixed-size kEntrySize SHM slot. */
	void
	sendFrameWithResolution(uint32_t width, uint32_t height, uint64_t tsNs)
	{
		size_t ySize = width * height;
		size_t uvSize = width * (height / 2);
		CU_ASSERT_FATAL(ySize + uvSize <= kEntrySize);

		uint32_t slot = mNextSlot++ % kNumEntries;
		uint8_t *dst = mShmAddr + kEntrySize * slot;
		memset(dst, 0x80, ySize);
		memset(dst + ySize, 0x80, uvSize);

		auto *frame = static_cast<struct vipc_frame *>(
			calloc(1, sizeof(struct vipc_frame)));
		CU_ASSERT_PTR_NOT_NULL_FATAL(frame);
		frame->format = VACQ_PIX_FORMAT_NV12;
		frame->ts_sof_ns = tsNs;
		frame->ts_eof_ns = tsNs;
		frame->width = width;
		frame->height = height;
		frame->num_planes = 2;
		frame->planes[0].stride = width;
		frame->planes[0].size = ySize;
		frame->planes[1].stride = width;
		frame->planes[1].size = uvSize;
		frame->crop.left = 0.f;
		frame->crop.top = 0.f;
		frame->crop.width = 1.f;
		frame->crop.height = 1.f;

		int ret = vipcs_send_frame(mCtx, frame, &slot, this);
		CU_ASSERT_EQUAL(ret, 0);
	}

	/* Triggers VipcSource::eosCb() on the client side, reason
	 * VIPC_EOS_REASON_NONE. */
	void sendEos()
	{
		int ret = vipcs_send_eos(mCtx);
		CU_ASSERT_EQUAL(ret, 0);
	}

	/* Same as sendEos(), with an explicit reason -- lets tests reach the
	 * VIPC_EOS_REASON_RESTART / VIPC_EOS_REASON_CONFIGURATION branches of
	 * VipcSource::eosCb(), not just the default NONE one covered by
	 * sendEos(). */
	void sendEosWithReason(enum vipc_eos_reason reason)
	{
		int ret = vipcs_send_eos_reason(mCtx, reason);
		CU_ASSERT_EQUAL(ret, 0);
	}

private:
	static struct vipc_status
	makeStatus(uint32_t framerateNum,
		   uint32_t framerateDen,
		   enum vacq_pix_format format = VACQ_PIX_FORMAT_NV12)
	{
		struct vipc_status status = {};
		status.format = format;
		status.width = kWidth;
		status.height = kHeight;
		if (format == VACQ_PIX_FORMAT_RAW16) {
			status.num_planes = 1;
			status.planes[0].stride = kWidth * 2;
			status.planes[0].size = kWidth * kHeight * 2;
		} else {
			status.num_planes = 2;
			status.planes[0].stride = kWidth;
			status.planes[0].size = kYSize;
			status.planes[1].stride = kWidth;
			status.planes[1].size = kUvSize;
		}
		status.framerate_num = framerateNum;
		status.framerate_den = framerateDen;
		return status;
	}

	static void
	runningRemotesCb(struct vipcs_ctx * /*ctx*/, int count, void *userdata)
	{
		static_cast<VipcTestServer *>(userdata)->mRunningRemotes.store(
			count);
	}

	static int frameReleaseCb(struct vipcs_ctx * /*ctx*/,
				  struct vipc_release *release,
				  void * /*userdata*/)
	{
		free(release->frame);
		return 0;
	}

	std::string mRelPath;
	std::string mAbsPath;
	int mShmFd = -1;
	uint8_t *mShmAddr = nullptr;
	struct vipc_be_ctx *mBeCtx = nullptr;
	struct vipcs_ctx *mCtx = nullptr;
	uint32_t mNextSlot = 0;
	std::atomic<int> mRunningRemotes{0};
};

} /* anonymous namespace */


/* Shared by every test below that needs a real (SHM-backed) VIPC source with
 * a raw video sink attached: creates the source, waits for readyToPlay and
 * the resulting raw video media, then attaches a sink whose listener
 * actually acknowledges flush/drain (see QueueDrainingRawVideoSinkListener
 * above for why g_stub_raw_video_sink_listener would not do). Does not call
 * play(): some callers (insertGreyFrame(), configure()) do not need it, and
 * callers that do just call it themselves right after this returns. */
static void
setupVipcSourceWithSink(TestPompLoop &loop,
			IPdraw *session,
			MediaTrackingListener &mediaListener,
			VipcSourceTrackingListener &sourceListener,
			QueueDrainingRawVideoSinkListener &sinkListener,
			const struct pdraw_vipc_source_params &sourceParams,
			IPdraw::IVipcSource **outSource,
			IPdraw::IRawVideoSink **outSink)
{
	int ret = session->createVipcSource(
		&sourceParams, &sourceListener, outSource);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(*outSource);

	bool gotReady = loop.pumpUntil(
		[&sourceListener]() { return sourceListener.mGotReadyToPlay; },
		15000);
	CU_ASSERT_TRUE_FATAL(gotReady);
	CU_ASSERT_TRUE_FATAL(sourceListener.mReady);

	bool gotMedia = loop.pumpUntil(
		[&mediaListener]() {
			return mediaListener.findRawVideoMedia() != nullptr;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotMedia);
	const MediaTrackingListener::Added *rawVideo =
		mediaListener.findRawVideoMedia();
	CU_ASSERT_PTR_NOT_NULL_FATAL(rawVideo);

	struct pdraw_video_sink_params sinkParams = {};
	ret = session->createRawVideoSink(
		rawVideo->id, &sinkParams, &sinkListener, outSink);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(*outSink);
	sinkListener.mQueue = (*outSink)->getQueue();
}


/* Complements pdraw_vipcsourcesink_test.c (libpdraw-backend/tests/), whose
 * client-side pattern this mirrors (create source -> wait ready -> play()
 * -> create a raw video sink on the resulting media -> pop real frames):
 * that reference program expects an external server; VipcTestServer above
 * supplies a genuine, self-contained SHM-backed one instead, so this is a
 * real end-to-end loopback -- not just a null-arg guard against a backend
 * that may or may not be compiled in (see test_api_vipc_source.cpp for
 * those). */
static void testCxxVipcSourceReceivesRealFrameFromShmServer()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	/* Abstract unix socket address (no filesystem entry, no leftover
	 * cleanup needed): both the server and pdraw's VipcSource client run
	 * on this same pomp_loop, so a single loop.pumpUntil() below drives
	 * both sides' socket I/O -- no threads, no separate loop to pump. */
	static const char *kVipcAddr = "unix:@pdraw_test_vipc_shm_server";
	VipcTestServer server(loop.raw(), kVipcAddr);

	struct pdraw_vipc_source_params sourceParams = {};
	sourceParams.address = kVipcAddr;
	sourceParams.backend_name = "shm";
	VipcSourceTrackingListener sourceListener;
	IPdraw::IVipcSource *source = nullptr;
	int ret = session->createVipcSource(
		&sourceParams, &sourceListener, &source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source);

	/* The server's status was sent before the client even existed (it is
	 * stored and auto-sent to newly-connecting clients), so this becomes
	 * true as soon as the client receives it and creates its output
	 * media -- before play() is ever called. */
	bool gotReady = loop.pumpUntil(
		[&sourceListener]() { return sourceListener.mGotReadyToPlay; },
		15000);
	CU_ASSERT_TRUE_FATAL(gotReady);
	CU_ASSERT_TRUE_FATAL(sourceListener.mReady);

	bool gotMedia = loop.pumpUntil(
		[&mediaListener]() {
			return mediaListener.findRawVideoMedia() != nullptr;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotMedia);
	const MediaTrackingListener::Added *rawVideo =
		mediaListener.findRawVideoMedia();
	CU_ASSERT_PTR_NOT_NULL_FATAL(rawVideo);
	ULOGI("rawVideo media: id=%u resolution=%ux%u (expected %ux%u)",
	      rawVideo->id,
	      rawVideo->videoInfo.resolution.width,
	      rawVideo->videoInfo.resolution.height,
	      VipcTestServer::kWidth,
	      VipcTestServer::kHeight);
	CU_ASSERT_EQUAL(rawVideo->videoInfo.resolution.width,
			VipcTestServer::kWidth);
	CU_ASSERT_EQUAL(rawVideo->videoInfo.resolution.height,
			VipcTestServer::kHeight);

	auto sourceOwner = std::unique_ptr<IPdraw::IVipcSource>(source);
	struct pdraw_video_sink_params sinkParams = {};
	IPdraw::IRawVideoSink *sink = nullptr;
	ret = session->createRawVideoSink(rawVideo->id,
					  &sinkParams,
					  &g_stub_raw_video_sink_listener,
					  &sink);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(sink);
	auto sinkOwner = std::unique_ptr<IPdraw::IRawVideoSink>(sink);

	struct mbuf_raw_video_frame_queue *outQueue = sink->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(outQueue);

	ret = source->play();
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	/* Wait for the server to see the client's play() (-> vipcc_start())
	 * request actually processed -- see the VipcTestServer class
	 * comment: sending before this would be silently dropped. */
	bool gotRunning = loop.pumpUntil(
		[&server]() { return server.runningRemoteCount() > 0; }, 15000);
	CU_ASSERT_TRUE_FATAL(gotRunning);

	/* Send exactly one real frame through the SHM server now that a
	 * running remote (the pdraw VipcSource client) is confirmed. */
	server.sendFrame(1000000);

	struct mbuf_raw_video_frame *frame = nullptr;
	bool gotFrame = loop.pumpUntil(
		[&]() {
			return mbuf_raw_video_frame_queue_pop(outQueue,
							      &frame) == 0;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotFrame);
	CU_ASSERT_PTR_NOT_NULL_FATAL(frame);

	/* The whole point: a real frame, sent by a real (SHM-backed) Video
	 * IPC server, round-tripped through VipcSource::processFrame() and
	 * came out the other end with the format/resolution genuinely
	 * propagated from the wire, not a placeholder. */
	struct vdef_raw_frame frameInfo = {};
	ret = mbuf_raw_video_frame_get_frame_info(frame, &frameInfo);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_TRUE(vdef_raw_format_cmp(&frameInfo.format, &vdef_nv12));
	CU_ASSERT_EQUAL(frameInfo.info.resolution.width,
			VipcTestServer::kWidth);
	CU_ASSERT_EQUAL(frameInfo.info.resolution.height,
			VipcTestServer::kHeight);
	mbuf_raw_video_frame_unref(frame);

	CU_ASSERT_FATAL(sourceListener.mFrameReadyCount.load() >= 1);

	/* Stop the session (and wait for it) BEFORE resetting the owners
	 * below, so that Session::asyncElementDelete() destroys the
	 * VipcSource/ExternalRawVideoSink elements -- and thus runs
	 * VipcSourceWrapper::clearElement()/RawVideoSinkWrapper::
	 * clearElement() -- while the wrappers are still alive. Resetting the
	 * wrappers first (as this test used to do) would instead run
	 * ~ElementWrapper() and the override would never execute. */
	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
	CU_ASSERT_EQUAL(mediaListener.mStopStatus, 0);

	CU_ASSERT_PTR_NULL(
		static_cast<VipcSourceWrapper *>(source)->getVipcSource());

	sinkOwner.reset();
	sourceOwner.reset();
}


/* pause() halts reception (drains the pipeline) and reports completion
 * asynchronously via vipcSourcePauseResponse() -- exercised here against a
 * running source with a real sink attached, since draining an unconnected
 * media completes trivially and would not exercise the channel round-trip
 * at all. */
static void testCxxVipcSourcePauseDrainsAndCallsPauseResponse()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	static const char *kVipcAddr = "unix:@pdraw_test_vipc_pause";
	VipcTestServer server(loop.raw(), kVipcAddr);

	struct pdraw_vipc_source_params sourceParams = {};
	sourceParams.address = kVipcAddr;
	sourceParams.backend_name = "shm";
	VipcSourceTrackingListener sourceListener;
	QueueDrainingRawVideoSinkListener sinkListener;
	IPdraw::IVipcSource *source = nullptr;
	IPdraw::IRawVideoSink *sink = nullptr;
	setupVipcSourceWithSink(loop,
				session,
				mediaListener,
				sourceListener,
				sinkListener,
				sourceParams,
				&source,
				&sink);
	auto sinkOwner = std::unique_ptr<IPdraw::IRawVideoSink>(sink);
	auto sourceOwner = std::unique_ptr<IPdraw::IVipcSource>(source);

	int ret = source->play();
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	bool gotRunning = loop.pumpUntil(
		[&server]() { return server.runningRemoteCount() > 0; }, 15000);
	CU_ASSERT_TRUE_FATAL(gotRunning);
	CU_ASSERT_FALSE(source->isPaused());

	/* VipcSource's FlushingState defaults to FLUSHED and is only ever
	 * flipped to UNFLUSHED by processFrame() once a real frame is
	 * successfully queued into an output channel: without this, flush()/
	 * drain() take the "already flushed, nothing to do" shortcut (see
	 * VipcSource::flush()) and never touch the channel at all --
	 * pauseResponse() would still fire (completeFlush() is idle-added
	 * either way), but onChannelDrained() would not. A real frame is sent
	 * here so the pause() below genuinely round-trips through the
	 * channel. */
	server.sendFrame(1000000);
	struct mbuf_raw_video_frame *primingFrame = nullptr;
	bool gotPrimingFrame = loop.pumpUntil(
		[&]() {
			return mbuf_raw_video_frame_queue_pop(
				       sinkListener.mQueue, &primingFrame) == 0;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotPrimingFrame);
	CU_ASSERT_PTR_NOT_NULL_FATAL(primingFrame);
	mbuf_raw_video_frame_unref(primingFrame);

	ret = source->pause();
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	/* The drain triggered by the call above is necessarily still pending
	 * at this point (it round-trips through the sink's channel, never
	 * synchronously): a second pause() must report -EALREADY rather than
	 * silently succeeding or dropping the first request. */
	int secondRet = source->pause();
	CU_ASSERT_EQUAL(secondRet, -EALREADY);

	bool gotPauseResponse = loop.pumpUntil(
		[&sourceListener]() {
			return sourceListener.mGotPauseResponse;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotPauseResponse);
	CU_ASSERT_TRUE(source->isPaused());

	/* Now that a real frame primed FlushingState to UNFLUSHED beforehand,
	 * pauseResponse() firing above really did round-trip through
	 * VipcSource::drain() -> channel->drain() ->
	 * QueueDrainingRawVideoSinkListener::onRawVideoSinkDrain() ->
	 * sk->queueDrained() -> VipcSource::onChannelDrained() ->
	 * completeFlush() -> pauseResponse(), rather than the "already
	 * flushed" shortcut. */
	CU_ASSERT(sinkListener.mDrainCount >= 1);

	sinkOwner.reset();
	sourceOwner.reset();

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
}


/* configure() is not implemented yet (see VipcSource::configure(): it is a
 * TODO that unconditionally returns -ENOSYS): must report -ENOSYS regardless
 * of arguments. */
static void testCxxVipcSourceConfigureReturnsNotImplemented()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	static const char *kVipcAddr = "unix:@pdraw_test_vipc_configure";
	VipcTestServer server(loop.raw(), kVipcAddr);

	struct pdraw_vipc_source_params sourceParams = {};
	sourceParams.address = kVipcAddr;
	sourceParams.backend_name = "shm";
	VipcSourceTrackingListener sourceListener;
	IPdraw::IVipcSource *source = nullptr;
	int ret = session->createVipcSource(
		&sourceParams, &sourceListener, &source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source);
	auto sourceOwner = std::unique_ptr<IPdraw::IVipcSource>(source);

	bool gotReady = loop.pumpUntil(
		[&sourceListener]() { return sourceListener.mGotReadyToPlay; },
		15000);
	CU_ASSERT_TRUE_FATAL(gotReady);

	ret = source->configure(nullptr, nullptr);
	CU_ASSERT_EQUAL(ret, -ENOSYS);

	struct vdef_dim resolution = {320, 240};
	struct vdef_rectf crop = {0.f, 0.f, 1.f, 1.f};
	ret = source->configure(&resolution, &crop);
	CU_ASSERT_EQUAL(ret, -ENOSYS);

	sourceOwner.reset();

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
}


/* insertGreyFrame() needs only mStatus (set once the status is received,
 * i.e. as soon as readyToPlay fires) -- no play() required -- and produces a
 * real frame tagged VDEF_FRAME_FLAG_FAKE at the configured resolution.
 * Timestamps must still be strictly monotonic like real frames. */
static void testCxxVipcSourceInsertGreyFrameProducesFakeFrame()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	static const char *kVipcAddr = "unix:@pdraw_test_vipc_grey";
	VipcTestServer server(loop.raw(), kVipcAddr);

	struct pdraw_vipc_source_params sourceParams = {};
	sourceParams.address = kVipcAddr;
	sourceParams.backend_name = "shm";
	VipcSourceTrackingListener sourceListener;
	QueueDrainingRawVideoSinkListener sinkListener;
	IPdraw::IVipcSource *source = nullptr;
	IPdraw::IRawVideoSink *sink = nullptr;
	setupVipcSourceWithSink(loop,
				session,
				mediaListener,
				sourceListener,
				sinkListener,
				sourceParams,
				&source,
				&sink);
	auto sinkOwner = std::unique_ptr<IPdraw::IRawVideoSink>(sink);
	auto sourceOwner = std::unique_ptr<IPdraw::IVipcSource>(source);

	int ret = source->insertGreyFrame(1000);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	struct mbuf_raw_video_frame *frame = nullptr;
	bool gotFrame = loop.pumpUntil(
		[&]() {
			return mbuf_raw_video_frame_queue_pop(
				       sinkListener.mQueue, &frame) == 0;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotFrame);
	CU_ASSERT_PTR_NOT_NULL_FATAL(frame);

	struct vdef_raw_frame frameInfo = {};
	ret = mbuf_raw_video_frame_get_frame_info(frame, &frameInfo);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_TRUE((frameInfo.info.flags & VDEF_FRAME_FLAG_FAKE) != 0);
	CU_ASSERT_EQUAL(frameInfo.info.resolution.width,
			VipcTestServer::kWidth);
	CU_ASSERT_EQUAL(frameInfo.info.resolution.height,
			VipcTestServer::kHeight);
	mbuf_raw_video_frame_unref(frame);

	/* Non-strictly-monotonic timestamps are rejected. */
	ret = source->insertGreyFrame(1000);
	CU_ASSERT_EQUAL(ret, -EINVAL);
	ret = source->insertGreyFrame(500);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	sinkOwner.reset();
	sourceOwner.reset();

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
}


/* setSessionMetadata()/getSessionMetadata() roundtrip, and the update
 * propagates downstream to a connected raw video sink. */
static void testCxxVipcSourceSessionMetadataRoundtrips()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	static const char *kVipcAddr = "unix:@pdraw_test_vipc_meta";
	VipcTestServer server(loop.raw(), kVipcAddr);

	struct pdraw_vipc_source_params sourceParams = {};
	sourceParams.address = kVipcAddr;
	sourceParams.backend_name = "shm";
	snprintf(sourceParams.session_meta.friendly_name,
		 sizeof(sourceParams.session_meta.friendly_name),
		 "pdraw_test_pipeline_vipc");
	VipcSourceTrackingListener sourceListener;
	QueueDrainingRawVideoSinkListener sinkListener;
	IPdraw::IVipcSource *source = nullptr;
	IPdraw::IRawVideoSink *sink = nullptr;
	setupVipcSourceWithSink(loop,
				session,
				mediaListener,
				sourceListener,
				sinkListener,
				sourceParams,
				&source,
				&sink);
	auto sinkOwner = std::unique_ptr<IPdraw::IRawVideoSink>(sink);
	auto sourceOwner = std::unique_ptr<IPdraw::IVipcSource>(source);

	int ret = source->setSessionMetadata(nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);
	ret = source->getSessionMetadata(nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	struct vmeta_session initialMeta = {};
	ret = source->getSessionMetadata(&initialMeta);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_STRING_EQUAL(initialMeta.friendly_name,
			       "pdraw_test_pipeline_vipc");

	struct vmeta_session newMeta = {};
	snprintf(newMeta.friendly_name,
		 sizeof(newMeta.friendly_name),
		 "updated_vipc_friendly_name");
	ret = source->setSessionMetadata(&newMeta);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	bool gotUpdate = loop.pumpUntil(
		[&sinkListener]() {
			return sinkListener.mGotSessionMetaUpdate;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotUpdate);
	CU_ASSERT_STRING_EQUAL(sinkListener.mLastSessionMeta.friendly_name,
			       "updated_vipc_friendly_name");

	struct vmeta_session updatedMeta = {};
	ret = source->getSessionMetadata(&updatedMeta);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_STRING_EQUAL(updatedMeta.friendly_name,
			       "updated_vipc_friendly_name");

	sinkOwner.reset();
	sourceOwner.reset();

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
}


/* An EOS sent by the server reaches VipcSource::eosCb(), which notifies the
 * listener and clears both the ready-to-play and running states. */
static void testCxxVipcSourceServerEosNotifiesListener()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	static const char *kVipcAddr = "unix:@pdraw_test_vipc_eos";
	VipcTestServer server(loop.raw(), kVipcAddr);

	struct pdraw_vipc_source_params sourceParams = {};
	sourceParams.address = kVipcAddr;
	sourceParams.backend_name = "shm";
	VipcSourceTrackingListener sourceListener;
	QueueDrainingRawVideoSinkListener sinkListener;
	IPdraw::IVipcSource *source = nullptr;
	IPdraw::IRawVideoSink *sink = nullptr;
	setupVipcSourceWithSink(loop,
				session,
				mediaListener,
				sourceListener,
				sinkListener,
				sourceParams,
				&source,
				&sink);
	auto sinkOwner = std::unique_ptr<IPdraw::IRawVideoSink>(sink);
	auto sourceOwner = std::unique_ptr<IPdraw::IVipcSource>(source);

	int ret = source->play();
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	bool gotRunning = loop.pumpUntil(
		[&server]() { return server.runningRemoteCount() > 0; }, 15000);
	CU_ASSERT_TRUE_FATAL(gotRunning);

	server.sendEos();

	bool gotEos = loop.pumpUntil(
		[&sourceListener]() { return sourceListener.mGotEos; }, 15000);
	CU_ASSERT_TRUE_FATAL(gotEos);
	CU_ASSERT_EQUAL(sourceListener.mLastEosReason,
			PDRAW_VIPC_SOURCE_EOS_REASON_NONE);
	CU_ASSERT_FALSE(source->isReadyToPlay());
	CU_ASSERT_TRUE(source->isPaused());

	sinkOwner.reset();
	sourceOwner.reset();

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
}


/* frame_timeout_ms fires when no frame is received for that long while
 * playing. VipcSource::onWatchdogTimer(), armed by play(), notifies the
 * listener via vipcSourceReadyToPlay(ready=false, reason=TIMEOUT), without
 * any EOS from the server. A short, explicit timeout is used instead of the
 * 2s default (DEFAULT_TIMEOUT_MS) to keep the test fast and deterministic. */
static void testCxxVipcSourceFrameWatchdogFiresOnTimeout()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	static const char *kVipcAddr = "unix:@pdraw_test_vipc_watchdog";
	VipcTestServer server(loop.raw(), kVipcAddr);

	struct pdraw_vipc_source_params sourceParams = {};
	sourceParams.address = kVipcAddr;
	sourceParams.backend_name = "shm";
	sourceParams.frame_timeout_ms = 300;
	VipcSourceTrackingListener sourceListener;
	QueueDrainingRawVideoSinkListener sinkListener;
	IPdraw::IVipcSource *source = nullptr;
	IPdraw::IRawVideoSink *sink = nullptr;
	setupVipcSourceWithSink(loop,
				session,
				mediaListener,
				sourceListener,
				sinkListener,
				sourceParams,
				&source,
				&sink);
	auto sinkOwner = std::unique_ptr<IPdraw::IRawVideoSink>(sink);
	auto sourceOwner = std::unique_ptr<IPdraw::IVipcSource>(source);

	int readyCountBeforePlay = sourceListener.mReadyToPlayCallCount;

	int ret = source->play();
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	bool gotRunning = loop.pumpUntil(
		[&server]() { return server.runningRemoteCount() > 0; }, 15000);
	CU_ASSERT_TRUE_FATAL(gotRunning);

	/* No frame is ever sent: the watchdog armed by play() must fire. */
	bool gotTimeout = loop.pumpUntil(
		[&]() {
			return sourceListener.mReadyToPlayCallCount >
			       readyCountBeforePlay;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotTimeout);
	CU_ASSERT_FALSE(sourceListener.mReady);
	CU_ASSERT_EQUAL(sourceListener.mLastReadyToPlayReason,
			PDRAW_VIPC_SOURCE_EOS_REASON_TIMEOUT);

	sinkOwner.reset();
	sourceOwner.reset();

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
}


/* Complements testCxxVipcSourceFrameWatchdogFiresOnTimeout (frame_timeout_ms,
 * "waiting for frames" while already running): connection_timeout_ms arms
 * the same watchdog in VipcSource::start() (since it is != -1), covering
 * the OTHER branch of onWatchdogTimer() instead -- "waiting for connection"
 * (mRunning is false, mVipcConnected is false; only the log message
 * distinguishes it internally from "waiting for status", but both are
 * observed identically from the outside: vipcSourceReadyToPlay(ready=false,
 * reason=TIMEOUT), never having been ready in the first place). No
 * VipcTestServer is created at all here: the address is deliberately never
 * listened on, so the connection genuinely never succeeds and the watchdog
 * is the only thing that can ever call the listener. A short, explicit
 * timeout is used instead of the 2s default (DEFAULT_TIMEOUT_MS) to keep
 * the test fast and deterministic, same as the frame_timeout_ms test. */
static void testCxxVipcSourceConnectionWatchdogFiresWhenNoServer()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	static const char *kVipcAddr = "unix:@pdraw_test_vipc_no_server";

	struct pdraw_vipc_source_params sourceParams = {};
	sourceParams.address = kVipcAddr;
	sourceParams.backend_name = "shm";
	sourceParams.connection_timeout_ms = 300;
	VipcSourceTrackingListener sourceListener;
	IPdraw::IVipcSource *source = nullptr;
	int ret = session->createVipcSource(
		&sourceParams, &sourceListener, &source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source);
	auto sourceOwner = std::unique_ptr<IPdraw::IVipcSource>(source);

	/* play() is deliberately never called: the connection watchdog fires
	 * on its own, armed directly by start(). */
	bool gotTimeout = loop.pumpUntil(
		[&sourceListener]() { return sourceListener.mGotReadyToPlay; },
		15000);
	CU_ASSERT_TRUE_FATAL(gotTimeout);
	CU_ASSERT_FALSE(sourceListener.mReady);
	CU_ASSERT_EQUAL(sourceListener.mLastReadyToPlayReason,
			PDRAW_VIPC_SOURCE_EOS_REASON_TIMEOUT);
	CU_ASSERT_FALSE(source->isReadyToPlay());

	sourceOwner.reset();

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
}


/* Shared setup for the 3 testCxxVipcSourceProcessFrameRejectsXxx tests
 * below: source only, no sink (vipcSourceFrameReady() fires regardless of
 * whether a sink is attached -- it is called from inside processFrame(),
 * before the per-output-channel queue() loop -- so a sink would only add
 * unrelated teardown surface). Returns once play() succeeded and the
 * server has seen the client's start() request (see VipcTestServer's
 * class comment: sending frames any earlier would be silently dropped). */
static void setupPlayingVipcSourceNoSink(
	TestPompLoop &loop,
	IPdraw *session,
	VipcTestServer &server,
	VipcSourceTrackingListener &sourceListener,
	const struct pdraw_vipc_source_params &sourceParams,
	IPdraw::IVipcSource **outSource)
{
	int ret = session->createVipcSource(
		&sourceParams, &sourceListener, outSource);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(*outSource);

	bool gotReady = loop.pumpUntil(
		[&sourceListener]() { return sourceListener.mGotReadyToPlay; },
		15000);
	CU_ASSERT_TRUE_FATAL(gotReady);

	ret = (*outSource)->play();
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	bool gotRunning = loop.pumpUntil(
		[&server]() { return server.runningRemoteCount() > 0; }, 15000);
	CU_ASSERT_TRUE_FATAL(gotRunning);
}


/* A frame tagged with a pixel format different from the output media's
 * (established from the status at media creation) is silently rejected by
 * VipcSource::processFrame()'s "invalid frame format" check (-EINVAL,
 * logged, no crash) -- observed here by the ABSENCE of a
 * vipcSourceFrameReady() call for it: a genuinely valid frame is sent right
 * after, to pumpUntil() on (a positive signal proving the rejected one was
 * actually processed, not merely still in-flight), then the total count is
 * checked to have grown by exactly 1 -- from the valid frame alone. */
static void testCxxVipcSourceProcessFrameRejectsWrongFormat()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	static const char *kVipcAddr = "unix:@pdraw_test_vipc_bad_format";
	VipcTestServer server(loop.raw(), kVipcAddr);

	struct pdraw_vipc_source_params sourceParams = {};
	sourceParams.address = kVipcAddr;
	sourceParams.backend_name = "shm";
	VipcSourceTrackingListener sourceListener;
	IPdraw::IVipcSource *source = nullptr;
	setupPlayingVipcSourceNoSink(
		loop, session, server, sourceListener, sourceParams, &source);
	auto sourceOwner = std::unique_ptr<IPdraw::IVipcSource>(source);

	int countBefore = sourceListener.mFrameReadyCount.load();

	/* NV21 instead of the media's NV12: a real, distinct
	 * vdef_raw_format (not an unmapped one), so this reaches the format
	 * *comparison*, not the format *conversion*. */
	server.sendFrameWithFormat(VACQ_PIX_FORMAT_NV21, 1000000);
	server.sendFrame(2000000);

	bool gotFrame = loop.pumpUntil(
		[&]() {
			return sourceListener.mFrameReadyCount.load() >
			       countBefore;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotFrame);
	CU_ASSERT_EQUAL(sourceListener.mFrameReadyCount.load(),
			countBefore + 1);

	sourceOwner.reset();

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
}


/* Same idea as testCxxVipcSourceProcessFrameRejectsWrongFormat, for the
 * "invalid frame resolution" check instead: a frame at a resolution
 * different from the output media's (kWidth/kHeight, established from the
 * status) is silently rejected. */
static void testCxxVipcSourceProcessFrameRejectsWrongResolution()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	static const char *kVipcAddr = "unix:@pdraw_test_vipc_bad_resolution";
	VipcTestServer server(loop.raw(), kVipcAddr);

	struct pdraw_vipc_source_params sourceParams = {};
	sourceParams.address = kVipcAddr;
	sourceParams.backend_name = "shm";
	VipcSourceTrackingListener sourceListener;
	IPdraw::IVipcSource *source = nullptr;
	setupPlayingVipcSourceNoSink(
		loop, session, server, sourceListener, sourceParams, &source);
	auto sourceOwner = std::unique_ptr<IPdraw::IVipcSource>(source);

	int countBefore = sourceListener.mFrameReadyCount.load();

	/* 32x32 instead of the media's 64x64. */
	server.sendFrameWithResolution(32, 32, 1000000);
	server.sendFrame(2000000);

	bool gotFrame = loop.pumpUntil(
		[&]() {
			return sourceListener.mFrameReadyCount.load() >
			       countBefore;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotFrame);
	CU_ASSERT_EQUAL(sourceListener.mFrameReadyCount.load(),
			countBefore + 1);

	sourceOwner.reset();

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
}


/* A frame whose timestamp is not strictly greater than the last
 * successfully processed one is silently rejected by
 * VipcSource::processFrame()'s "non-strictly-monotonic timestamp" check.
 * Sends 3 frames -- ok, then an earlier one (rejected), then a later one
 * again -- and checks exactly 2 (the 1st and 3rd) were ever reported ready,
 * proving the 2nd was dropped rather than merely delayed or silently
 * accepted out of order. */
static void testCxxVipcSourceProcessFrameRejectsNonMonotonicTimestamp()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	static const char *kVipcAddr = "unix:@pdraw_test_vipc_bad_timestamp";
	VipcTestServer server(loop.raw(), kVipcAddr);

	struct pdraw_vipc_source_params sourceParams = {};
	sourceParams.address = kVipcAddr;
	sourceParams.backend_name = "shm";
	VipcSourceTrackingListener sourceListener;
	IPdraw::IVipcSource *source = nullptr;
	setupPlayingVipcSourceNoSink(
		loop, session, server, sourceListener, sourceParams, &source);
	auto sourceOwner = std::unique_ptr<IPdraw::IVipcSource>(source);

	int countBefore = sourceListener.mFrameReadyCount.load();

	/* 1st frame: establishes VipcSource's mLastTimestamp. */
	server.sendFrame(1000000);
	bool gotFirstFrame = loop.pumpUntil(
		[&]() {
			return sourceListener.mFrameReadyCount.load() >
			       countBefore;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotFirstFrame);
	CU_ASSERT_EQUAL(sourceListener.mFrameReadyCount.load(),
			countBefore + 1);

	/* 2nd frame: earlier timestamp than the 1st -- rejected. 3rd frame:
	 * later again, to pumpUntil() on. */
	server.sendFrame(500000);
	server.sendFrame(2000000);

	bool gotThirdFrame = loop.pumpUntil(
		[&]() {
			return sourceListener.mFrameReadyCount.load() >
			       countBefore + 1;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotThirdFrame);
	CU_ASSERT_EQUAL(sourceListener.mFrameReadyCount.load(),
			countBefore + 2);

	sourceOwner.reset();

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
}


/* A new status differing only by framerate reaches VipcSource::statusCb(),
 * which calls vipcSourceFramerateChanged() before deciding whether to keep
 * the current media (listener returns true) or recreate it (returns false,
 * the default in VipcSourceTrackingListener here): recreation is exercised
 * since it is the only path with an externally observable effect (a new
 * media, at the new framerate). No sink is attached: the media being
 * destroyed and replaced mid-test is exactly what is under test, so keeping
 * a sink around would only add an unrelated teardown edge case. */
static void testCxxVipcSourceFramerateChangeRecreatesMedia()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	static const char *kVipcAddr = "unix:@pdraw_test_vipc_framerate";
	VipcTestServer server(loop.raw(), kVipcAddr);

	struct pdraw_vipc_source_params sourceParams = {};
	sourceParams.address = kVipcAddr;
	sourceParams.backend_name = "shm";
	VipcSourceTrackingListener sourceListener;
	IPdraw::IVipcSource *source = nullptr;
	int ret = session->createVipcSource(
		&sourceParams, &sourceListener, &source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source);
	auto sourceOwner = std::unique_ptr<IPdraw::IVipcSource>(source);

	bool gotReady = loop.pumpUntil(
		[&sourceListener]() { return sourceListener.mGotReadyToPlay; },
		15000);
	CU_ASSERT_TRUE_FATAL(gotReady);

	bool gotMedia = loop.pumpUntil(
		[&mediaListener]() {
			return mediaListener.findRawVideoMedia() != nullptr;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotMedia);
	const MediaTrackingListener::Added *firstMedia =
		mediaListener.findRawVideoMedia();
	CU_ASSERT_PTR_NOT_NULL_FATAL(firstMedia);
	unsigned int firstMediaId = firstMedia->id;
	CU_ASSERT_EQUAL(firstMedia->videoInfo.framerate.num, 30);
	CU_ASSERT_EQUAL(firstMedia->videoInfo.framerate.den, 1);

	server.sendStatusWithFramerate(25, 1);

	bool gotFramerateChanged = loop.pumpUntil(
		[&sourceListener]() {
			return sourceListener.mGotFramerateChanged;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotFramerateChanged);
	CU_ASSERT_EQUAL(sourceListener.mPrevFramerate.num, 30);
	CU_ASSERT_EQUAL(sourceListener.mPrevFramerate.den, 1);
	CU_ASSERT_EQUAL(sourceListener.mNewFramerate.num, 25);
	CU_ASSERT_EQUAL(sourceListener.mNewFramerate.den, 1);

	/* A new media, distinct from the first, must appear at the new
	 * framerate. */
	const MediaTrackingListener::Added *newMedia = nullptr;
	bool gotNewMedia = loop.pumpUntil(
		[&]() {
			for (const auto &a : mediaListener.mAdded) {
				if (a.videoFormat == VDEF_FRAME_TYPE_RAW &&
				    a.id != firstMediaId) {
					newMedia = &a;
					return true;
				}
			}
			return false;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotNewMedia);
	CU_ASSERT_PTR_NOT_NULL_FATAL(newMedia);
	CU_ASSERT_EQUAL(newMedia->videoInfo.framerate.num, 25);
	CU_ASSERT_EQUAL(newMedia->videoInfo.framerate.den, 1);

	sourceOwner.reset();

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
}


/* mem_implem != MBUF_MEM_IMPLEM_TYPE_AUTO makes VipcSource::start() go
 * through isMbufMemImplemSupported(): MBUF_MEM_IMPLEM_TYPE_GENERIC is the
 * only implem the "shm" VIPC backend accepts (see the BackendType::SHM case
 * in isMbufMemImplemSupported(), pdraw_vipc_source.cpp), and this build
 * enables CONFIG_ALCHEMY_BUILD_LIBMEDIA_BUFFERS_MEMORY_GENERIC (confirmed in
 * out/groundsdk-linux/global.config), so mbuf_get_supported_implems() is
 * guaranteed to list it: start() must succeed and the source must reach
 * readyToPlay exactly like the default (AUTO) case. */
static void testCxxVipcSourceMemImplemGenericAcceptedForShmBackend()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	static const char *kVipcAddr = "unix:@pdraw_test_vipc_mem_generic";
	VipcTestServer server(loop.raw(), kVipcAddr);

	struct pdraw_vipc_source_params sourceParams = {};
	sourceParams.address = kVipcAddr;
	sourceParams.backend_name = "shm";
	sourceParams.mem_implem = MBUF_MEM_IMPLEM_TYPE_GENERIC;
	VipcSourceTrackingListener sourceListener;
	IPdraw::IVipcSource *source = nullptr;
	int ret = session->createVipcSource(
		&sourceParams, &sourceListener, &source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source);
	auto sourceOwner = std::unique_ptr<IPdraw::IVipcSource>(source);

	bool gotReady = loop.pumpUntil(
		[&sourceListener]() { return sourceListener.mGotReadyToPlay; },
		15000);
	CU_ASSERT_TRUE_FATAL(gotReady);
	CU_ASSERT_TRUE(sourceListener.mReady);

	sourceOwner.reset();

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
}


/* MBUF_MEM_IMPLEM_TYPE_SHM (the raw shared-memory mbuf implem) is a real,
 * supported mbuf mem implem in this build
 * (CONFIG_ALCHEMY_BUILD_LIBMEDIA_BUFFERS_MEMORY_SHM=y) but is not the one
 * the "shm" VIPC backend accepts (only MBUF_MEM_IMPLEM_TYPE_GENERIC is, see
 * isMbufMemImplemSupported()'s BackendType::SHM case): this exercises the
 * "found by mbuf_get_supported_implems() but rejected by the
 * backend-specific switch" branch, distinct from "not found at all".
 * VipcSource::start() must fail with -EPROTO and createVipcSource() must
 * propagate it without ever handing back a live source. */
static void testCxxVipcSourceMemImplemMismatchRejectedByBackend()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	static const char *kVipcAddr = "unix:@pdraw_test_vipc_mem_mismatch";
	VipcTestServer server(loop.raw(), kVipcAddr);

	struct pdraw_vipc_source_params sourceParams = {};
	sourceParams.address = kVipcAddr;
	sourceParams.backend_name = "shm";
	sourceParams.mem_implem = MBUF_MEM_IMPLEM_TYPE_SHM;
	VipcSourceTrackingListener sourceListener;
	IPdraw::IVipcSource *source = nullptr;
	int ret = session->createVipcSource(
		&sourceParams, &sourceListener, &source);
	CU_ASSERT_EQUAL(ret, -EPROTO);
	CU_ASSERT_PTR_NULL(source);

	/* The failed element's own VipcSource::start() error path already
	 * calls its internal stop() synchronously (no media/channels ever
	 * existed), so it is fully torn down; session->stop() just needs to
	 * be able to complete normally around that leftover stopped
	 * element. */
	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
}


/* Re-sending the exact same status VipcSource already holds (same
 * resolution/format/framerate as the constructor's initial send) reaches
 * statusCb()'s "ignored (identical)" branch (vipc_status_cmp() true): no new
 * media may appear and the stored status must be left untouched. A
 * genuinely different status is sent right after so the test has a
 * deterministic event (vipcSourceFramerateChanged()) to pumpUntil() on,
 * proving the identical resend was actually processed (not just still
 * in-flight) before checking that it left no trace. */
static void testCxxVipcSourceIdenticalStatusResendIgnored()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	static const char *kVipcAddr = "unix:@pdraw_test_vipc_ident_status";
	VipcTestServer server(loop.raw(), kVipcAddr);

	struct pdraw_vipc_source_params sourceParams = {};
	sourceParams.address = kVipcAddr;
	sourceParams.backend_name = "shm";
	VipcSourceTrackingListener sourceListener;
	IPdraw::IVipcSource *source = nullptr;
	int ret = session->createVipcSource(
		&sourceParams, &sourceListener, &source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source);
	auto sourceOwner = std::unique_ptr<IPdraw::IVipcSource>(source);

	bool gotReady = loop.pumpUntil(
		[&sourceListener]() { return sourceListener.mGotReadyToPlay; },
		15000);
	CU_ASSERT_TRUE_FATAL(gotReady);

	bool gotMedia = loop.pumpUntil(
		[&mediaListener]() {
			return mediaListener.findRawVideoMedia() != nullptr;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotMedia);
	size_t mediaCountBeforeResend = mediaListener.mAdded.size();

	/* Identical to the constructor's initial makeStatus(30, 1): reaches
	 * the "ignored (identical)" branch, not "differs only by
	 * framerate". */
	server.sendStatusWithFramerate(30, 1);

	/* A genuinely different status right after, to pumpUntil() on. */
	server.sendStatusWithFramerate(25, 1);
	bool gotFramerateChanged = loop.pumpUntil(
		[&sourceListener]() {
			return sourceListener.mGotFramerateChanged;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotFramerateChanged);

	/* The identical resend must not have left any trace: the listener
	 * saw exactly ONE framerate-change call, comparing the ORIGINAL
	 * 30/1 against 25/1 -- not some value already mutated by the
	 * identical resend, and not a second spurious call. */
	CU_ASSERT_EQUAL(sourceListener.mFramerateChangedCallCount, 1);
	CU_ASSERT_EQUAL(sourceListener.mPrevFramerate.num, 30);
	CU_ASSERT_EQUAL(sourceListener.mPrevFramerate.den, 1);
	CU_ASSERT_EQUAL(sourceListener.mNewFramerate.num, 25);
	CU_ASSERT_EQUAL(sourceListener.mNewFramerate.den, 1);

	/* Nor must it have created any extra media of its own: only the
	 * framerate-change recreation (one extra entry) is expected. */
	bool gotNewMedia = loop.pumpUntil(
		[&]() {
			return mediaListener.mAdded.size() >
			       mediaCountBeforeResend;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotNewMedia);
	CU_ASSERT_EQUAL(mediaListener.mAdded.size(),
			mediaCountBeforeResend + 1);

	sourceOwner.reset();

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
}


/* Complements testCxxVipcSourceIdenticalStatusResendIgnored: that test never
 * calls play(), so mRunning is false when the identical resend arrives and
 * statusCb()'s "ignore:" label (l. 1684) finds self->mWasRunning false,
 * skipping its body entirely. Here the source is actively running
 * (play() was called first) when the identical resend arrives, so
 * statusCb() captures mWasRunning=true right before "goto ignore" -- l.
 * 1685-1697's "if (self->mWasRunning)" then re-starts the vipc client and
 * re-arms the watchdog with connection_timeout_ms.
 * frame_timeout_ms is disabled (-1) so play() never arms a watchdog of its
 * own: the only watchdog that can ever fire here is the one (re-)armed by
 * that restart body, so observing a TIMEOUT notification after
 * connection_timeout_ms is direct proof the branch executed. */
static void testCxxVipcSourceIdenticalStatusResendWhileRunningRearmsWatchdog()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	static const char *kVipcAddr = "unix:@pdraw_test_vipc_ident_running";
	VipcTestServer server(loop.raw(), kVipcAddr);

	struct pdraw_vipc_source_params sourceParams = {};
	sourceParams.address = kVipcAddr;
	sourceParams.backend_name = "shm";
	sourceParams.frame_timeout_ms = -1;
	sourceParams.connection_timeout_ms = 300;
	VipcSourceTrackingListener sourceListener;
	QueueDrainingRawVideoSinkListener sinkListener;
	IPdraw::IVipcSource *source = nullptr;
	IPdraw::IRawVideoSink *sink = nullptr;
	setupVipcSourceWithSink(loop,
				session,
				mediaListener,
				sourceListener,
				sinkListener,
				sourceParams,
				&source,
				&sink);
	auto sinkOwner = std::unique_ptr<IPdraw::IRawVideoSink>(sink);
	auto sourceOwner = std::unique_ptr<IPdraw::IVipcSource>(source);

	int ret = source->play();
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	bool gotRunning = loop.pumpUntil(
		[&server]() { return server.runningRemoteCount() > 0; }, 15000);
	CU_ASSERT_TRUE_FATAL(gotRunning);

	int readyCountBeforeResend = sourceListener.mReadyToPlayCallCount;

	/* Identical to the constructor's initial makeStatus(30, 1), still
	 * held unchanged in mStatus (no EOS/reconfiguration happened yet). */
	server.sendStatusWithFramerate(30, 1);

	bool gotTimeout = loop.pumpUntil(
		[&]() {
			return sourceListener.mReadyToPlayCallCount >
			       readyCountBeforeResend;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotTimeout);
	CU_ASSERT_EQUAL(sourceListener.mLastReadyToPlayReason,
			PDRAW_VIPC_SOURCE_EOS_REASON_TIMEOUT);
	CU_ASSERT_FALSE(sourceListener.mReady);

	sinkOwner.reset();
	sourceOwner.reset();

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
}


/* Complements testCxxVipcSourceFramerateChangeRecreatesMedia (listener
 * returns false -> recreate media): here the listener returns true
 * ("ignore"), the other branch of statusCb()'s framerate-change handling.
 * No new media may appear. The ignored framerate is nonetheless folded into
 * VipcSource's stored status in place (see the ignore branch in
 * VipcSource::statusCb(): "self->mStatus->framerate_num = ..."), confirmed
 * here by re-sending that SAME new framerate a second time and observing it
 * now take the "identical" path (no extra vipcSourceFramerateChanged() call)
 * instead of "differs" again. */
static void testCxxVipcSourceFramerateChangeIgnoredKeepsMedia()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	static const char *kVipcAddr = "unix:@pdraw_test_vipc_framerate_ignore";
	VipcTestServer server(loop.raw(), kVipcAddr);

	struct pdraw_vipc_source_params sourceParams = {};
	sourceParams.address = kVipcAddr;
	sourceParams.backend_name = "shm";
	VipcSourceTrackingListener sourceListener;
	sourceListener.mFramerateChangeIgnoreResult = true;
	IPdraw::IVipcSource *source = nullptr;
	int ret = session->createVipcSource(
		&sourceParams, &sourceListener, &source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source);
	auto sourceOwner = std::unique_ptr<IPdraw::IVipcSource>(source);

	bool gotReady = loop.pumpUntil(
		[&sourceListener]() { return sourceListener.mGotReadyToPlay; },
		15000);
	CU_ASSERT_TRUE_FATAL(gotReady);

	bool gotMedia = loop.pumpUntil(
		[&mediaListener]() {
			return mediaListener.findRawVideoMedia() != nullptr;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotMedia);
	size_t mediaCountBefore = mediaListener.mAdded.size();

	server.sendStatusWithFramerate(25, 1);
	bool gotFramerateChanged = loop.pumpUntil(
		[&sourceListener]() {
			return sourceListener.mGotFramerateChanged;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotFramerateChanged);
	CU_ASSERT_EQUAL(sourceListener.mFramerateChangedCallCount, 1);
	CU_ASSERT_EQUAL(sourceListener.mPrevFramerate.num, 30);
	CU_ASSERT_EQUAL(sourceListener.mPrevFramerate.den, 1);
	CU_ASSERT_EQUAL(sourceListener.mNewFramerate.num, 25);
	CU_ASSERT_EQUAL(sourceListener.mNewFramerate.den, 1);

	/* The ignore branch never calls createMedia()/setupMedia(): no
	 * onMediaAdded() will ever be scheduled for it, so there is nothing
	 * to wait for -- unlike the "recreate" test, this negative check can
	 * be made right away. */
	CU_ASSERT_EQUAL(mediaListener.mAdded.size(), mediaCountBefore);

	/* Re-sending the SAME (25, 1) framerate must now be seen as
	 * "identical" (the ignore branch above updated the stored status to
	 * 25/1 in place), not as a second "differs" event: sending a further,
	 * genuinely different framerate right after gives a deterministic
	 * event to pumpUntil() on, and the surviving call count/values prove
	 * the (25, 1) resend produced no call of its own. */
	server.sendStatusWithFramerate(25, 1);
	server.sendStatusWithFramerate(20, 1);
	bool gotSecondChange = loop.pumpUntil(
		[&sourceListener]() {
			return sourceListener.mFramerateChangedCallCount > 1;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotSecondChange);
	CU_ASSERT_EQUAL(sourceListener.mFramerateChangedCallCount, 2);
	CU_ASSERT_EQUAL(sourceListener.mPrevFramerate.num, 25);
	CU_ASSERT_EQUAL(sourceListener.mNewFramerate.num, 20);

	sourceOwner.reset();

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
}


/* Same shape as testCxxVipcSourceServerEosNotifiesListener (reason NONE),
 * but for VIPC_EOS_REASON_RESTART -> PDRAW_VIPC_SOURCE_EOS_REASON_RESTART:
 * the RESTART/CONFIGURATION branches of eosCb()'s reason switch send a
 * different downstream event internally (RECONFIGURE instead of EOS, and
 * RESTART alone skips the mFirstFrame reset) but the directly observable,
 * listener-facing behavior under test here is the reason value itself. */
static void testCxxVipcSourceEosRestartReasonPropagates()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	static const char *kVipcAddr = "unix:@pdraw_test_vipc_eos_restart";
	VipcTestServer server(loop.raw(), kVipcAddr);

	struct pdraw_vipc_source_params sourceParams = {};
	sourceParams.address = kVipcAddr;
	sourceParams.backend_name = "shm";
	VipcSourceTrackingListener sourceListener;
	QueueDrainingRawVideoSinkListener sinkListener;
	IPdraw::IVipcSource *source = nullptr;
	IPdraw::IRawVideoSink *sink = nullptr;
	setupVipcSourceWithSink(loop,
				session,
				mediaListener,
				sourceListener,
				sinkListener,
				sourceParams,
				&source,
				&sink);
	auto sinkOwner = std::unique_ptr<IPdraw::IRawVideoSink>(sink);
	auto sourceOwner = std::unique_ptr<IPdraw::IVipcSource>(source);

	int ret = source->play();
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	bool gotRunning = loop.pumpUntil(
		[&server]() { return server.runningRemoteCount() > 0; }, 15000);
	CU_ASSERT_TRUE_FATAL(gotRunning);

	server.sendEosWithReason(VIPC_EOS_REASON_RESTART);

	bool gotEos = loop.pumpUntil(
		[&sourceListener]() { return sourceListener.mGotEos; }, 15000);
	CU_ASSERT_TRUE_FATAL(gotEos);
	CU_ASSERT_EQUAL(sourceListener.mLastEosReason,
			PDRAW_VIPC_SOURCE_EOS_REASON_RESTART);
	CU_ASSERT_FALSE(source->isReadyToPlay());
	CU_ASSERT_TRUE(source->isPaused());

	sinkOwner.reset();
	sourceOwner.reset();

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
}


/* Same as testCxxVipcSourceEosRestartReasonPropagates, for
 * VIPC_EOS_REASON_CONFIGURATION -> PDRAW_VIPC_SOURCE_EOS_REASON_CONFIGURATION
 * (the third and last branch of eosCb()'s reason switch). */
static void testCxxVipcSourceEosConfigurationReasonPropagates()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	static const char *kVipcAddr =
		"unix:@pdraw_test_vipc_eos_configuration";
	VipcTestServer server(loop.raw(), kVipcAddr);

	struct pdraw_vipc_source_params sourceParams = {};
	sourceParams.address = kVipcAddr;
	sourceParams.backend_name = "shm";
	VipcSourceTrackingListener sourceListener;
	QueueDrainingRawVideoSinkListener sinkListener;
	IPdraw::IVipcSource *source = nullptr;
	IPdraw::IRawVideoSink *sink = nullptr;
	setupVipcSourceWithSink(loop,
				session,
				mediaListener,
				sourceListener,
				sinkListener,
				sourceParams,
				&source,
				&sink);
	auto sinkOwner = std::unique_ptr<IPdraw::IRawVideoSink>(sink);
	auto sourceOwner = std::unique_ptr<IPdraw::IVipcSource>(source);

	int ret = source->play();
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	bool gotRunning = loop.pumpUntil(
		[&server]() { return server.runningRemoteCount() > 0; }, 15000);
	CU_ASSERT_TRUE_FATAL(gotRunning);

	server.sendEosWithReason(VIPC_EOS_REASON_CONFIGURATION);

	bool gotEos = loop.pumpUntil(
		[&sourceListener]() { return sourceListener.mGotEos; }, 15000);
	CU_ASSERT_TRUE_FATAL(gotEos);
	CU_ASSERT_EQUAL(sourceListener.mLastEosReason,
			PDRAW_VIPC_SOURCE_EOS_REASON_CONFIGURATION);
	CU_ASSERT_FALSE(source->isReadyToPlay());
	CU_ASSERT_TRUE(source->isPaused());

	sinkOwner.reset();
	sourceOwner.reset();

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
}


/* VipcSource::eosCb() invalidates the current status and calls
 * setupMedia(), which -- since mOutputMedia still exists and mKeepMedia is
 * false (VipcSourceTrackingListener::vipcSourceEndOfStream() always returns
 * false) -- calls flush(discard=true) (VipcSource::setupMedia()): a genuine
 * discard-flush of the still-live output channel, reaching
 * VipcSource::onChannelFlushed() once the sink acknowledges it.
 *
 * This is deliberately NOT tested via VipcSource::stop() (tried first,
 * confirmed broken by a real run): ExternalRawVideoSink::stop()
 * (pdraw_external_raw_video_sink.cpp) unconditionally nulls its own
 * mVideoSinkListener BEFORE any pending flush notification is delivered
 * ("we do not want to call listener functions any more" once the sink
 * itself is being torn down, per its own comment) -- so a flush triggered
 * by VipcSource::stop() with the sink stopping concurrently (as
 * session->stop() does, whichever element order) never reaches
 * IRawVideoSink::Listener::onRawVideoSinkFlush(): the pending notification
 * is idle-deferred, and ExternalRawVideoSink::stop() (called moments later,
 * same call stack) always wins the race and nulls the listener first --
 * ExternalRawVideoSink::callVideoSinkFlush() then just self-completes via
 * flushDone() without ever calling out to the app. VipcSource::
 * onChannelFlushed() itself IS still reached in that scenario (flushDone()
 * -> channel->flushDone() round-trips back to it regardless), but
 * mFlushCount -- the only externally-observable proxy available for it --
 * would stay 0 either way, making stop() a fundamentally unusable vehicle
 * for this particular assertion. Sending an EOS instead keeps the sink
 * fully alive and its listener valid throughout, so the notification
 * genuinely arrives. */
static void testCxxVipcSourceEosTriggeredFlushReachesSinkListener()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	static const char *kVipcAddr = "unix:@pdraw_test_vipc_eos_flush";
	VipcTestServer server(loop.raw(), kVipcAddr);

	struct pdraw_vipc_source_params sourceParams = {};
	sourceParams.address = kVipcAddr;
	sourceParams.backend_name = "shm";
	VipcSourceTrackingListener sourceListener;
	QueueDrainingRawVideoSinkListener sinkListener;
	IPdraw::IVipcSource *source = nullptr;
	IPdraw::IRawVideoSink *sink = nullptr;
	setupVipcSourceWithSink(loop,
				session,
				mediaListener,
				sourceListener,
				sinkListener,
				sourceParams,
				&source,
				&sink);
	auto sinkOwner = std::unique_ptr<IPdraw::IRawVideoSink>(sink);
	auto sourceOwner = std::unique_ptr<IPdraw::IVipcSource>(source);

	int ret = source->play();
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	bool gotRunning = loop.pumpUntil(
		[&server]() { return server.runningRemoteCount() > 0; }, 15000);
	CU_ASSERT_TRUE_FATAL(gotRunning);

	/* Both VipcSource's and ExternalRawVideoSink's FlushingState default
	 * to FLUSHED and only leave it once a real frame is queued/received
	 * (see the "already flushed"/"already flushed or drained" shortcuts
	 * in VipcSource::flush() and ExternalRawVideoSink::flush()):
	 * otherwise the EOS-triggered flush below would self-complete on
	 * either side without the notification ever reaching the sink's
	 * listener. */
	server.sendFrame(1000000);
	struct mbuf_raw_video_frame *primingFrame = nullptr;
	bool gotPrimingFrame = loop.pumpUntil(
		[&]() {
			return mbuf_raw_video_frame_queue_pop(
				       sinkListener.mQueue, &primingFrame) == 0;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotPrimingFrame);
	CU_ASSERT_PTR_NOT_NULL_FATAL(primingFrame);
	mbuf_raw_video_frame_unref(primingFrame);

	server.sendEos();

	bool gotFlush = loop.pumpUntil(
		[&sinkListener]() { return sinkListener.mFlushCount >= 1; },
		15000);
	CU_ASSERT_TRUE_FATAL(gotFlush);

	sinkOwner.reset();
	sourceOwner.reset();

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
}


/* C API shim coverage: exercises the 5 PdrawVipcSourceListener methods that
 * require a live VIPC server (configured, play_resp, frame_ready, pause_resp,
 * end_of_stream).  Uses the SHM loopback (VipcTestServer) and a dedicated
 * struct pdraw * (created via pdraw_new()) so the C API callbacks can be
 * exercised in the same file where VipcTestServer is defined.
 *
 * framerate_changed is wired up but not exercised here (would need a
 * sendStatusWithFramerate() call -- same mechanism as
 * testCxxVipcSourceFramerateChangeRecreatesMedia). */
static void testCVipcSourceListenerCbsWithShmServer()
{
	TestPompLoop loop;

	static const char *kVipcAddr = "unix:@pdraw_test_vipc_capi_cbs";
	VipcTestServer server(loop.raw(), kVipcAddr);

	struct Ud {
		int configuredCount = 0;
		int readyToPlayCount = 0;
		bool lastReady = false;
		int playRespCount = 0;
		int pauseRespCount = 0;
		int frameReadyCount = 0;
		int endOfStreamCount = 0;
		int framerateChangedCount = 0;
		int stopRespCount = 0;
	} ud;

	struct pdraw_cbs sessionCbs = {};
	sessionCbs.stop_resp = [](struct pdraw *, int, void *userdata) {
		static_cast<Ud *>(userdata)->stopRespCount++;
	};

	struct pdraw *p = nullptr;
	int ret = pdraw_new(loop.raw(), &sessionCbs, &ud, &p);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(p);

	struct pdraw_vipc_source_params params = {};
	params.address = kVipcAddr;
	params.backend_name = "shm";

	struct pdraw_vipc_source_cbs cbs = {};
	cbs.ready_to_play = [](struct pdraw *,
			       struct pdraw_vipc_source *,
			       int ready,
			       enum pdraw_vipc_source_eos_reason,
			       void *userdata) {
		auto *u = static_cast<Ud *>(userdata);
		u->lastReady = (bool)ready;
		u->readyToPlayCount++;
	};
	cbs.configured = [](struct pdraw *,
			    struct pdraw_vipc_source *,
			    int /*status*/,
			    const struct vdef_format_info *,
			    const struct vdef_rectf *,
			    void *userdata) {
		static_cast<Ud *>(userdata)->configuredCount++;
	};
	cbs.play_resp =
		[](struct pdraw *, struct pdraw_vipc_source *, void *userdata) {
			static_cast<Ud *>(userdata)->playRespCount++;
		};
	cbs.pause_resp =
		[](struct pdraw *, struct pdraw_vipc_source *, void *userdata) {
			static_cast<Ud *>(userdata)->pauseRespCount++;
		};
	cbs.framerate_changed = [](struct pdraw *,
				   struct pdraw_vipc_source *,
				   const struct vdef_frac *,
				   const struct vdef_frac *,
				   void *userdata) -> bool {
		static_cast<Ud *>(userdata)->framerateChangedCount++;
		return false;
	};
	cbs.frame_ready = [](struct pdraw *,
			     struct pdraw_vipc_source *,
			     struct mbuf_raw_video_frame *,
			     void *userdata) {
		static_cast<Ud *>(userdata)->frameReadyCount++;
	};
	cbs.end_of_stream = [](struct pdraw *,
			       struct pdraw_vipc_source *,
			       enum pdraw_vipc_source_eos_reason,
			       void *userdata) -> bool {
		static_cast<Ud *>(userdata)->endOfStreamCount++;
		return false;
	};

	struct pdraw_vipc_source *src = nullptr;
	ret = pdraw_vipc_source_new(p, &params, &cbs, &ud, &src);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(src);

	/* configured + ready_to_play(true): fire as soon as the client receives
	 * the server's initial status (sent at VipcTestServer construction). */
	bool gotReady = loop.pumpUntil(
		[&ud]() {
			return ud.configuredCount > 0 &&
			       ud.readyToPlayCount > 0;
		},
		5000);
	CU_ASSERT_TRUE(gotReady);
	CU_ASSERT_TRUE(ud.lastReady);

	/* framerate_changed shim: sending a new status with a different fps
	 * (25 vs. the initial 30) reaches statusCb()'s "differs only by
	 * framerate" path → vipcSourceFramerateChanged fires; returning false
	 * destroys and recreates the media → a second configured fires.
	 * Note: ready_to_play does NOT re-fire here — createMedia()'s
	 * "if (!mReady)" guard stays false because mReady is never reset in
	 * the statusCb path (only eosCb resets it). */
	server.sendStatusWithFramerate(25, 1);
	bool gotFrChange = loop.pumpUntil(
		[&ud]() { return ud.framerateChangedCount > 0; }, 5000);
	CU_ASSERT_TRUE(gotFrChange);
	bool gotNewConfigured = loop.pumpUntil(
		[&ud]() { return ud.configuredCount >= 2; }, 5000);
	CU_ASSERT_TRUE(gotNewConfigured);

	/* play() → vipcc_start() → mRunning=true → playResponse() idle-added
	 * → callPlayResponse() → play_resp shim */
	ret = pdraw_vipc_source_play(p, src);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	bool gotRunning = loop.pumpUntil(
		[&server]() { return server.runningRemoteCount() > 0; }, 5000);
	CU_ASSERT_TRUE_FATAL(gotRunning);
	bool gotPlayResp =
		loop.pumpUntil([&ud]() { return ud.playRespCount > 0; }, 5000);
	CU_ASSERT_TRUE(gotPlayResp);

	/* Server sends a real SHM frame → processFrame() → frame_ready shim.
	 * vipcSourceFrameReady fires before the per-output-channel loop, so no
	 * downstream sink is needed. */
	server.sendFrame(1000000);
	bool gotFrameReady = loop.pumpUntil(
		[&ud]() { return ud.frameReadyCount > 0; }, 5000);
	CU_ASSERT_TRUE(gotFrameReady);

	/* EOS while playing → end_of_stream shim (return false = destroy
	 * media). eosCb: mRunning=false, mReady=false → vipcSourceEndOfStream →
	 * mStatus=nullptr → setupMedia() → flush() → completeFlush() →
	 * destroyMedia()
	 * → vipcSourceReadyToPlay(false, ...) on subsequent loop iterations. */
	server.sendEos();
	bool gotEos = loop.pumpUntil(
		[&ud]() { return ud.endOfStreamCount > 0; }, 5000);
	CU_ASSERT_TRUE(gotEos);

	/* Wait for ready_to_play(false) from destroyMedia() to confirm the
	 * EOS-triggered flush has fully completed and FlushingState is FLUSHED.
	 * Required before pause() so drain() does not race the ongoing flush.
	 * Checking !lastReady (not a fixed count) because the framerate_changed
	 * step above already added extra ready_to_play(true) calls. */
	bool gotNotReady = loop.pumpUntil(
		[&ud]() { return ud.endOfStreamCount > 0 && !ud.lastReady; },
		5000);
	CU_ASSERT_TRUE(gotNotReady);

	/* pause() with mReady=false, mRunning=false: skips vipcc_stop, calls
	 * drain() → flush() FLUSHED shortcut → idle → completeFlush()
	 * → mPausePending → pauseResponse() idle → callPauseResponse()
	 * → pause_resp shim */
	ret = pdraw_vipc_source_pause(p, src);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	bool gotPauseResp =
		loop.pumpUntil([&ud]() { return ud.pauseRespCount > 0; }, 5000);
	CU_ASSERT_TRUE(gotPauseResp);

	/* ~VipcSourceWrapper() calls stop() which sets
	 * mVipcSourceListener=nullptr (no UAF), then pdraw_stop() lets the
	 * session drain all elements. */
	pdraw_vipc_source_destroy(p, src);
	ret = pdraw_stop(p);
	CU_ASSERT_EQUAL(ret, 0);
	bool gotStop =
		loop.pumpUntil([&ud]() { return ud.stopRespCount > 0; }, 5000);
	CU_ASSERT_TRUE(gotStop);
	pdraw_destroy(p);
}


/* VipcSource::eosCb()'s "Notify not ready-to-play if eos is ignored" branch:
 * when the listener's vipcSourceEndOfStream() returns true (mKeepMedia), the
 * output media is NOT torn down (unlike
 * testCxxVipcSourceServerEosNotifiesListener's default/false case, where
 * destroyMedia() eventually fires its own, separate ready-to-play(false)
 * notification once the async flush completes) -- so the only place left
 * that can tell the app playback stopped being ready is this synchronous
 * branch, taken right inside eosCb() itself, immediately after the
 * listener's vipcSourceEndOfStream() call returns. */
static void testCxxVipcSourceEosIgnoredNotifiesNotReady()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	static const char *kVipcAddr = "unix:@pdraw_test_vipc_eos_ignored";
	VipcTestServer server(loop.raw(), kVipcAddr);

	struct pdraw_vipc_source_params sourceParams = {};
	sourceParams.address = kVipcAddr;
	sourceParams.backend_name = "shm";
	VipcSourceTrackingListener sourceListener;
	sourceListener.mEosIgnoreResult = true;
	QueueDrainingRawVideoSinkListener sinkListener;
	IPdraw::IVipcSource *source = nullptr;
	IPdraw::IRawVideoSink *sink = nullptr;
	setupVipcSourceWithSink(loop,
				session,
				mediaListener,
				sourceListener,
				sinkListener,
				sourceParams,
				&source,
				&sink);
	auto sinkOwner = std::unique_ptr<IPdraw::IRawVideoSink>(sink);
	auto sourceOwner = std::unique_ptr<IPdraw::IVipcSource>(source);

	/* setupVipcSourceWithSink() already waited for the initial
	 * ready_to_play(true) triggered by output media creation. */
	int readyCountBeforeEos = sourceListener.mReadyToPlayCallCount;
	CU_ASSERT_TRUE_FATAL(sourceListener.mReady);

	int ret = source->play();
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	bool gotRunning = loop.pumpUntil(
		[&server]() { return server.runningRemoteCount() > 0; }, 15000);
	CU_ASSERT_TRUE_FATAL(gotRunning);

	server.sendEos();

	bool gotNotReady = loop.pumpUntil(
		[&]() {
			return sourceListener.mReadyToPlayCallCount >
			       readyCountBeforeEos;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotNotReady);
	CU_ASSERT_EQUAL(sourceListener.mReadyToPlayCallCount,
			readyCountBeforeEos + 1);
	CU_ASSERT_FALSE(sourceListener.mReady);
	CU_ASSERT_EQUAL(sourceListener.mLastReadyToPlayReason,
			PDRAW_VIPC_SOURCE_EOS_REASON_NONE);
	CU_ASSERT_TRUE(sourceListener.mGotEos);
	CU_ASSERT_EQUAL(sourceListener.mLastEosReason,
			PDRAW_VIPC_SOURCE_EOS_REASON_NONE);
	CU_ASSERT_FALSE(source->isReadyToPlay());
	CU_ASSERT_TRUE(source->isPaused());

	sinkOwner.reset();
	sourceOwner.reset();

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
}


/* VipcSource::createMedia()'s "if (mWasRunning && mRunning)" (l. 1468-1481):
 * a belt-and-suspenders restart, distinct from the identical one in
 * statusCb()'s "ignore:" label (see
 * testCxxVipcSourceIdenticalStatusResendWhileRunningRearmsWatchdog). Reaching
 * it needs mWasRunning (captured true by eosCb() because play() had been
 * called) AND mRunning true AT THE SAME TIME inside this single
 * createMedia() call -- but eosCb() itself always resets mRunning to false,
 * and nothing else sets it back to true except a fresh play() call, which
 * requires mReady, false since that same eosCb() call. The only way both end
 * up true together is a *reentrant* play() call, made from inside the very
 * ready_to_play(true) notification createMedia() fires a few lines above (l.
 * 1456-1466) once the kept media (mKeepMedia, via mEosIgnoreResult) is
 * reconfigured after the EOS -- exactly what a real app auto-resuming
 * playback on "ready" would do. sourceListener.mAutoPlayOnReady arms that.
 * frame_timeout_ms is disabled (-1) so neither play() call arms a watchdog
 * of its own: the only watchdog that can ever fire here is the one
 * (re-)armed by createMedia()'s restart body, so a TIMEOUT notification
 * after connection_timeout_ms is direct proof it executed. */
static void testCxxVipcSourceCreateMediaRestartsRunningAfterIgnoredEos()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	static const char *kVipcAddr = "unix:@pdraw_test_vipc_create_media";
	VipcTestServer server(loop.raw(), kVipcAddr);

	struct pdraw_vipc_source_params sourceParams = {};
	sourceParams.address = kVipcAddr;
	sourceParams.backend_name = "shm";
	sourceParams.frame_timeout_ms = -1;
	sourceParams.connection_timeout_ms = 300;
	VipcSourceTrackingListener sourceListener;
	sourceListener.mEosIgnoreResult = true;
	QueueDrainingRawVideoSinkListener sinkListener;
	IPdraw::IVipcSource *source = nullptr;
	IPdraw::IRawVideoSink *sink = nullptr;
	setupVipcSourceWithSink(loop,
				session,
				mediaListener,
				sourceListener,
				sinkListener,
				sourceParams,
				&source,
				&sink);
	auto sinkOwner = std::unique_ptr<IPdraw::IRawVideoSink>(sink);
	auto sourceOwner = std::unique_ptr<IPdraw::IVipcSource>(source);

	int ret = source->play();
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	bool gotRunning = loop.pumpUntil(
		[&server]() { return server.runningRemoteCount() > 0; }, 15000);
	CU_ASSERT_TRUE_FATAL(gotRunning);

	/* Armed only now: setupVipcSourceWithSink()'s own initial
	 * ready(true) notification must not trigger a reentrant play(). */
	sourceListener.mAutoPlayOnReady = true;

	server.sendEos();

	/* eosCb()'s own synchronous ready(false) notification (the ignored-EOS
	 * branch tested by testCxxVipcSourceEosIgnoredNotifiesNotReady). */
	bool gotNotReady =
		loop.pumpUntil([&]() { return !sourceListener.mReady; }, 15000);
	CU_ASSERT_TRUE_FATAL(gotNotReady);

	/* A fresh status on the kept media: createMedia() takes the
	 * "goto media_created" shortcut, fires ready(true) (triggering the
	 * listener's reentrant play() while mWasRunning is still set from the
	 * EOS above), then reaches l. 1468. */
	server.sendStatusWithFramerate(30, 1);

	bool gotReadyAgain = loop.pumpUntil(
		[&sourceListener]() { return sourceListener.mReady; }, 15000);
	CU_ASSERT_TRUE_FATAL(gotReadyAgain);

	int readyCountBeforeTimeout = sourceListener.mReadyToPlayCallCount;

	bool gotTimeout = loop.pumpUntil(
		[&]() {
			return sourceListener.mReadyToPlayCallCount >
			       readyCountBeforeTimeout;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotTimeout);
	CU_ASSERT_EQUAL(sourceListener.mLastReadyToPlayReason,
			PDRAW_VIPC_SOURCE_EOS_REASON_TIMEOUT);
	CU_ASSERT_FALSE(sourceListener.mReady);

	sinkOwner.reset();
	sourceOwner.reset();

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
}


static void testCxxVipcSourcePlayMultipleTimesNoop()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	static const char *kVipcAddr = "unix:@pdraw_test_vipc_play_multi";
	VipcTestServer server(loop.raw(), kVipcAddr);

	struct pdraw_vipc_source_params sourceParams = {};
	sourceParams.address = kVipcAddr;
	sourceParams.backend_name = "shm";
	VipcSourceTrackingListener sourceListener;
	QueueDrainingRawVideoSinkListener sinkListener;
	IPdraw::IVipcSource *source = nullptr;
	IPdraw::IRawVideoSink *sink = nullptr;
	setupVipcSourceWithSink(loop,
				session,
				mediaListener,
				sourceListener,
				sinkListener,
				sourceParams,
				&source,
				&sink);
	auto sinkOwner = std::unique_ptr<IPdraw::IRawVideoSink>(sink);
	auto sourceOwner = std::unique_ptr<IPdraw::IVipcSource>(source);

	int ret = source->play();
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	/* Call play again while running (mRunning = true) -> no-op returning 0
	 */
	ret = source->play();
	CU_ASSERT_EQUAL(ret, 0);

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);

	sinkOwner.reset();
	sourceOwner.reset();
}


static void testCxxVipcSourcePlayWhilePausePendingReturnsEbusy()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	static const char *kVipcAddr = "unix:@pdraw_test_vipc_pause_pending";
	VipcTestServer server(loop.raw(), kVipcAddr);

	struct pdraw_vipc_source_params sourceParams = {};
	sourceParams.address = kVipcAddr;
	sourceParams.backend_name = "shm";
	VipcSourceTrackingListener sourceListener;
	QueueDrainingRawVideoSinkListener sinkListener;
	IPdraw::IVipcSource *source = nullptr;
	IPdraw::IRawVideoSink *sink = nullptr;
	setupVipcSourceWithSink(loop,
				session,
				mediaListener,
				sourceListener,
				sinkListener,
				sourceParams,
				&source,
				&sink);
	auto sinkOwner = std::unique_ptr<IPdraw::IRawVideoSink>(sink);
	auto sourceOwner = std::unique_ptr<IPdraw::IVipcSource>(source);

	int ret = source->play();
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	/* Trigger pause */
	ret = source->pause();
	CU_ASSERT_EQUAL(ret, 0);

	/* Immediately call play() while pause is pending -> -EBUSY */
	ret = source->play();
	CU_ASSERT_EQUAL(ret, -EBUSY);

	/* Call pause() again while pause is pending -> -EALREADY */
	ret = source->pause();
	CU_ASSERT_EQUAL(ret, -EALREADY);

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);

	sinkOwner.reset();
	sourceOwner.reset();
}


static void testCxxVipcSourcePlayWhenNotReadyReturnsEproto()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	static const char *kVipcAddr = "unix:@pdraw_test_vipc_not_ready";
	struct pdraw_vipc_source_params sourceParams = {};
	sourceParams.address = kVipcAddr;
	sourceParams.backend_name = "shm";
	VipcSourceTrackingListener sourceListener;
	IPdraw::IVipcSource *source = nullptr;
	int ret = session->createVipcSource(
		&sourceParams, &sourceListener, &source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	auto sourceOwner = std::unique_ptr<IPdraw::IVipcSource>(source);

	/* Before server is connected / ready, play() returns -EPROTO */
	ret = source->play();
	CU_ASSERT_EQUAL(ret, -EPROTO);

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);

	sourceOwner.reset();
}


static void testCxxVipcSourceInsertGreyFrameRaw16()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	static const char *kVipcAddr = "unix:@pdraw_test_vipc_grey_raw16";
	VipcTestServer server(loop.raw(), kVipcAddr);
	server.sendStatusWithFormat(VACQ_PIX_FORMAT_RAW16);

	struct pdraw_vipc_source_params sourceParams = {};
	sourceParams.address = kVipcAddr;
	sourceParams.backend_name = "shm";
	VipcSourceTrackingListener sourceListener;
	QueueDrainingRawVideoSinkListener sinkListener;
	IPdraw::IVipcSource *source = nullptr;
	IPdraw::IRawVideoSink *sink = nullptr;
	setupVipcSourceWithSink(loop,
				session,
				mediaListener,
				sourceListener,
				sinkListener,
				sourceParams,
				&source,
				&sink);
	auto sinkOwner = std::unique_ptr<IPdraw::IRawVideoSink>(sink);
	auto sourceOwner = std::unique_ptr<IPdraw::IVipcSource>(source);

	int ret = source->insertGreyFrame(1000);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	struct mbuf_raw_video_frame *frame = nullptr;
	bool gotFrame = loop.pumpUntil(
		[&]() {
			return mbuf_raw_video_frame_queue_pop(
				       sinkListener.mQueue, &frame) == 0;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotFrame);
	CU_ASSERT_PTR_NOT_NULL_FATAL(frame);

	struct vdef_raw_frame frameInfo = {};
	ret = mbuf_raw_video_frame_get_frame_info(frame, &frameInfo);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_TRUE((frameInfo.info.flags & VDEF_FRAME_FLAG_FAKE) != 0);
	CU_ASSERT_TRUE(vdef_raw_format_cmp(&frameInfo.format, &vdef_raw16));

	mbuf_raw_video_frame_unref(frame);

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);

	sinkOwner.reset();
	sourceOwner.reset();
}


static void testCxxVipcSourceInsertGreyFrameUnsupportedFormat()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	static const char *kVipcAddr = "unix:@pdraw_test_vipc_grey_unsupp";
	VipcTestServer server(loop.raw(), kVipcAddr);
	server.sendStatusWithFormat(VACQ_PIX_FORMAT_BAYER_BGGR);

	struct pdraw_vipc_source_params sourceParams = {};
	sourceParams.address = kVipcAddr;
	sourceParams.backend_name = "shm";
	VipcSourceTrackingListener sourceListener;
	QueueDrainingRawVideoSinkListener sinkListener;
	IPdraw::IVipcSource *source = nullptr;
	IPdraw::IRawVideoSink *sink = nullptr;
	setupVipcSourceWithSink(loop,
				session,
				mediaListener,
				sourceListener,
				sinkListener,
				sourceParams,
				&source,
				&sink);
	auto sinkOwner = std::unique_ptr<IPdraw::IRawVideoSink>(sink);
	auto sourceOwner = std::unique_ptr<IPdraw::IVipcSource>(source);

	int ret = source->insertGreyFrame(1000);
	CU_ASSERT_EQUAL(ret, -ENOSYS);

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);

	sinkOwner.reset();
	sourceOwner.reset();
}


static void testCxxVipcSourceMaxPushedFrameCountAutoPauses()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	static const char *kVipcAddr = "unix:@pdraw_test_vipc_max_pushed";
	VipcTestServer server(loop.raw(), kVipcAddr);

	struct pdraw_vipc_source_params sourceParams = {};
	sourceParams.address = kVipcAddr;
	sourceParams.backend_name = "shm";
	sourceParams.max_pushed_frame_count = 1;
	VipcSourceTrackingListener sourceListener;
	QueueDrainingRawVideoSinkListener sinkListener;
	IPdraw::IVipcSource *source = nullptr;
	IPdraw::IRawVideoSink *sink = nullptr;
	setupVipcSourceWithSink(loop,
				session,
				mediaListener,
				sourceListener,
				sinkListener,
				sourceParams,
				&source,
				&sink);
	auto sinkOwner = std::unique_ptr<IPdraw::IRawVideoSink>(sink);
	auto sourceOwner = std::unique_ptr<IPdraw::IVipcSource>(source);

	int ret = source->play();
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	bool gotRunning = loop.pumpUntil(
		[&server]() { return server.runningRemoteCount() > 0; }, 15000);
	CU_ASSERT_TRUE_FATAL(gotRunning);

	/* Server pushes frame #1 */
	server.sendFrame(1000000000ULL);

	/* Source auto-pauses after 1 frame */
	bool gotPaused = loop.pumpUntil(
		[&source]() { return source->isPaused(); }, 15000);
	CU_ASSERT_TRUE(gotPaused);

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);

	sinkOwner.reset();
	sourceOwner.reset();
}


static void testCxxVipcSourceCustomTimescale()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	static const char *kVipcAddr = "unix:@pdraw_test_vipc_timescale";
	VipcTestServer server(loop.raw(), kVipcAddr);

	struct pdraw_vipc_source_params sourceParams = {};
	sourceParams.address = kVipcAddr;
	sourceParams.backend_name = "shm";
	sourceParams.timescale = 90000;
	VipcSourceTrackingListener sourceListener;
	QueueDrainingRawVideoSinkListener sinkListener;
	IPdraw::IVipcSource *source = nullptr;
	IPdraw::IRawVideoSink *sink = nullptr;
	setupVipcSourceWithSink(loop,
				session,
				mediaListener,
				sourceListener,
				sinkListener,
				sourceParams,
				&source,
				&sink);
	auto sinkOwner = std::unique_ptr<IPdraw::IRawVideoSink>(sink);
	auto sourceOwner = std::unique_ptr<IPdraw::IVipcSource>(source);

	int ret = source->play();
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	bool gotRunning = loop.pumpUntil(
		[&server]() { return server.runningRemoteCount() > 0; }, 15000);
	CU_ASSERT_TRUE_FATAL(gotRunning);

	server.sendFrame(1000000000ULL);

	struct mbuf_raw_video_frame *frame = nullptr;
	bool gotFrame = loop.pumpUntil(
		[&]() {
			return mbuf_raw_video_frame_queue_pop(
				       sinkListener.mQueue, &frame) == 0;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotFrame);
	CU_ASSERT_PTR_NOT_NULL_FATAL(frame);

	struct vdef_raw_frame frameInfo = {};
	ret = mbuf_raw_video_frame_get_frame_info(frame, &frameInfo);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(frameInfo.info.timescale, 90000);

	mbuf_raw_video_frame_unref(frame);

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);

	sinkOwner.reset();
	sourceOwner.reset();
}


static void testCxxVipcSourceSmallTimescaleScaledUp()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	static const char *kVipcAddr = "unix:@pdraw_test_vipc_timescale_small";
	VipcTestServer server(loop.raw(), kVipcAddr);

	struct pdraw_vipc_source_params sourceParams = {};
	sourceParams.address = kVipcAddr;
	sourceParams.backend_name = "shm";
	sourceParams.timescale =
		100; /* < 1000 -> mTimescale *= 1000 -> 100000 */
	VipcSourceTrackingListener sourceListener;
	QueueDrainingRawVideoSinkListener sinkListener;
	IPdraw::IVipcSource *source = nullptr;
	IPdraw::IRawVideoSink *sink = nullptr;
	setupVipcSourceWithSink(loop,
				session,
				mediaListener,
				sourceListener,
				sinkListener,
				sourceParams,
				&source,
				&sink);
	auto sinkOwner = std::unique_ptr<IPdraw::IRawVideoSink>(sink);
	auto sourceOwner = std::unique_ptr<IPdraw::IVipcSource>(source);

	int ret = source->play();
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	bool gotRunning = loop.pumpUntil(
		[&server]() { return server.runningRemoteCount() > 0; }, 15000);
	CU_ASSERT_TRUE_FATAL(gotRunning);

	server.sendFrame(1000000000ULL);

	struct mbuf_raw_video_frame *frame = nullptr;
	bool gotFrame = loop.pumpUntil(
		[&]() {
			return mbuf_raw_video_frame_queue_pop(
				       sinkListener.mQueue, &frame) == 0;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotFrame);
	CU_ASSERT_PTR_NOT_NULL_FATAL(frame);

	struct vdef_raw_frame frameInfo = {};
	ret = mbuf_raw_video_frame_get_frame_info(frame, &frameInfo);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(frameInfo.info.timescale, 100000);

	mbuf_raw_video_frame_unref(frame);

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);

	sinkOwner.reset();
	sourceOwner.reset();
}


static void testCxxVipcSourceDecimation()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	static const char *kVipcAddr = "unix:@pdraw_test_vipc_decimation";
	VipcTestServer server(loop.raw(), kVipcAddr);

	struct pdraw_vipc_source_params sourceParams = {};
	sourceParams.address = kVipcAddr;
	sourceParams.backend_name = "shm";
	sourceParams.decimation = 2;
	VipcSourceTrackingListener sourceListener;
	QueueDrainingRawVideoSinkListener sinkListener;
	IPdraw::IVipcSource *source = nullptr;
	IPdraw::IRawVideoSink *sink = nullptr;
	setupVipcSourceWithSink(loop,
				session,
				mediaListener,
				sourceListener,
				sinkListener,
				sourceParams,
				&source,
				&sink);
	auto sinkOwner = std::unique_ptr<IPdraw::IRawVideoSink>(sink);
	auto sourceOwner = std::unique_ptr<IPdraw::IVipcSource>(source);

	int ret = source->play();
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	bool gotRunning = loop.pumpUntil(
		[&server]() { return server.runningRemoteCount() > 0; }, 15000);
	CU_ASSERT_TRUE_FATAL(gotRunning);

	/* Send 4 frames */
	for (int i = 1; i <= 4; i++)
		server.sendFrame(i * 100000000ULL);

	/* Wait for frames to be processed */
	(void)loop.pumpUntil([]() { return false; }, 500);

	/* Decimation 2 means 1st frame pushed, 2nd dropped, 3rd pushed, 4th
	 * dropped -> 2 frames pushed */
	int count = 0;
	struct mbuf_raw_video_frame *frame = nullptr;
	while (mbuf_raw_video_frame_queue_pop(sinkListener.mQueue, &frame) ==
	       0) {
		count++;
		mbuf_raw_video_frame_unref(frame);
	}
	CU_ASSERT_EQUAL(count, 2);

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);

	sinkOwner.reset();
	sourceOwner.reset();
}


static void testCxxVipcSourceFriendlyName()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	static const char *kVipcAddr = "unix:@pdraw_test_vipc_friendly";
	VipcTestServer server(loop.raw(), kVipcAddr);

	struct pdraw_vipc_source_params sourceParams = {};
	sourceParams.address = kVipcAddr;
	sourceParams.backend_name = "shm";
	sourceParams.friendly_name = "FrontCamVipcSource";
	VipcSourceTrackingListener sourceListener;
	IPdraw::IVipcSource *source = nullptr;
	int ret = session->createVipcSource(
		&sourceParams, &sourceListener, &source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	auto sourceOwner = std::unique_ptr<IPdraw::IVipcSource>(source);

	bool gotReady = loop.pumpUntil(
		[&sourceListener]() { return sourceListener.mGotReadyToPlay; },
		15000);
	CU_ASSERT_TRUE_FATAL(gotReady);

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);

	sourceOwner.reset();
}


static void testCxxVipcSourceCustomCrop()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	static const char *kVipcAddr = "unix:@pdraw_test_vipc_custom_crop";
	VipcTestServer server(loop.raw(), kVipcAddr);

	struct pdraw_vipc_source_params sourceParams = {};
	sourceParams.address = kVipcAddr;
	sourceParams.backend_name = "shm";
	sourceParams.crop.left = 0.1f;
	sourceParams.crop.top = 0.2f;
	sourceParams.crop.width = 0.8f;
	sourceParams.crop.height = 0.7f;

	class CropCheckingListener : public VipcSourceTrackingListener {
	public:
		void
		vipcSourceConfigured(IPdraw *p,
				     IPdraw::IVipcSource *src,
				     int status,
				     const struct vdef_format_info *info,
				     const struct vdef_rectf *crop) override
		{
			VipcSourceTrackingListener::vipcSourceConfigured(
				p, src, status, info, crop);
			if (crop != nullptr)
				mReceivedCrop = *crop;
			mGotConfigured = true;
		}
		bool mGotConfigured = false;
		struct vdef_rectf mReceivedCrop = {};
	};

	CropCheckingListener sourceListener;
	IPdraw::IVipcSource *source = nullptr;
	int ret = session->createVipcSource(
		&sourceParams, &sourceListener, &source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	auto sourceOwner = std::unique_ptr<IPdraw::IVipcSource>(source);

	bool gotConfigured = loop.pumpUntil(
		[&sourceListener]() { return sourceListener.mGotConfigured; },
		15000);
	CU_ASSERT_TRUE_FATAL(gotConfigured);
	CU_ASSERT_DOUBLE_EQUAL(sourceListener.mReceivedCrop.left, 0.1f, 1e-4);
	CU_ASSERT_DOUBLE_EQUAL(sourceListener.mReceivedCrop.top, 0.2f, 1e-4);
	CU_ASSERT_DOUBLE_EQUAL(sourceListener.mReceivedCrop.width, 0.8f, 1e-4);
	CU_ASSERT_DOUBLE_EQUAL(sourceListener.mReceivedCrop.height, 0.7f, 1e-4);

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);

	sourceOwner.reset();
}


static void testCxxVipcSourceInsertGreyFrameNullStatusReturnsEproto()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	static const char *kVipcAddr = "unix:@pdraw_test_vipc_grey_null_status";
	struct pdraw_vipc_source_params sourceParams = {};
	sourceParams.address = kVipcAddr;
	sourceParams.backend_name = "shm";
	VipcSourceTrackingListener sourceListener;
	IPdraw::IVipcSource *source = nullptr;
	int ret = session->createVipcSource(
		&sourceParams, &sourceListener, &source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	auto sourceOwner = std::unique_ptr<IPdraw::IVipcSource>(source);

	/* insertGreyFrame with null mStatus -> -EPROTO */
	ret = source->insertGreyFrame(1000);
	CU_ASSERT_EQUAL(ret, -EPROTO);

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);

	sourceOwner.reset();
}


static void testCxxVipcSourceStopWithInFlightFrame()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	static const char *kVipcAddr = "unix:@pdraw_test_vipc_in_flight";
	VipcTestServer server(loop.raw(), kVipcAddr);

	struct pdraw_vipc_source_params sourceParams = {};
	sourceParams.address = kVipcAddr;
	sourceParams.backend_name = "shm";
	VipcSourceTrackingListener sourceListener;
	QueueDrainingRawVideoSinkListener sinkListener;
	IPdraw::IVipcSource *source = nullptr;
	IPdraw::IRawVideoSink *sink = nullptr;
	setupVipcSourceWithSink(loop,
				session,
				mediaListener,
				sourceListener,
				sinkListener,
				sourceParams,
				&source,
				&sink);
	auto sinkOwner = std::unique_ptr<IPdraw::IRawVideoSink>(sink);
	auto sourceOwner = std::unique_ptr<IPdraw::IVipcSource>(source);

	int ret = source->play();
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	bool gotRunning = loop.pumpUntil(
		[&server]() { return server.runningRemoteCount() > 0; }, 15000);
	CU_ASSERT_TRUE_FATAL(gotRunning);

	server.sendFrame(1000000000ULL);

	/* Pop frame from queue but DO NOT unref it immediately (keep it in
	 * flight) */
	struct mbuf_raw_video_frame *frame = nullptr;
	bool gotFrame = loop.pumpUntil(
		[&]() {
			return mbuf_raw_video_frame_queue_pop(
				       sinkListener.mQueue, &frame) == 0;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotFrame);
	CU_ASSERT_PTR_NOT_NULL_FATAL(frame);

	/* Stop session while frame is still held in-flight (mUsedFrameCount >
	 * 0) */
	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);

	/* Unref the frame now (triggers releaseFrameCb ->
	 * decrementUsedFrameCount -> idleCompleteStop) */
	mbuf_raw_video_frame_unref(frame);

	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);

	sinkOwner.reset();
	sourceOwner.reset();
}


static void testCxxVipcSourceProcessFrameRejectsWrongFullRange()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	static const char *kVipcAddr = "unix:@pdraw_test_vipc_wrong_range";
	VipcTestServer server(loop.raw(), kVipcAddr);

	struct pdraw_vipc_source_params sourceParams = {};
	sourceParams.address = kVipcAddr;
	sourceParams.backend_name = "shm";
	VipcSourceTrackingListener sourceListener;
	QueueDrainingRawVideoSinkListener sinkListener;
	IPdraw::IVipcSource *source = nullptr;
	IPdraw::IRawVideoSink *sink = nullptr;
	setupVipcSourceWithSink(loop,
				session,
				mediaListener,
				sourceListener,
				sinkListener,
				sourceParams,
				&source,
				&sink);
	auto sinkOwner = std::unique_ptr<IPdraw::IRawVideoSink>(sink);
	auto sourceOwner = std::unique_ptr<IPdraw::IVipcSource>(source);

	int ret = source->play();
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	bool gotRunning = loop.pumpUntil(
		[&server]() { return server.runningRemoteCount() > 0; }, 15000);
	CU_ASSERT_TRUE_FATAL(gotRunning);

	/* Status full_range is 0 (limited range). Send frame with full_range =
	 * 1 */
	server.sendFrameWithFullRange(1, 1000000000ULL);

	/* Wait briefly: frame is dropped by processFrame due to wrong range */
	(void)loop.pumpUntil([]() { return false; }, 500);

	/* Queue remains empty */
	struct mbuf_raw_video_frame *frame = nullptr;
	CU_ASSERT_NOT_EQUAL(
		mbuf_raw_video_frame_queue_pop(sinkListener.mQueue, &frame), 0);

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);

	sinkOwner.reset();
	sourceOwner.reset();
}


static void testCxxVipcSourceInsertGreyFrameInFrameReadyUpdatesIndex()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	static const char *kVipcAddr = "unix:@pdraw_test_vipc_grey_in_cb";
	VipcTestServer server(loop.raw(), kVipcAddr);

	struct pdraw_vipc_source_params sourceParams = {};
	sourceParams.address = kVipcAddr;
	sourceParams.backend_name = "shm";

	class InsertGreyOnFrameReadyListener
			: public VipcSourceTrackingListener {
	public:
		void vipcSourceFrameReady(
			IPdraw *p,
			IPdraw::IVipcSource *src,
			struct mbuf_raw_video_frame *frame) override
		{
			VipcSourceTrackingListener::vipcSourceFrameReady(
				p, src, frame);
			if (mInsertOnce && src != nullptr) {
				mInsertOnce = false;
				/* Insert grey frame with timestamp after
				 * initial frame */
				(void)src->insertGreyFrame(2000000);
			}
		}
		bool mInsertOnce = true;
	};

	InsertGreyOnFrameReadyListener sourceListener;
	QueueDrainingRawVideoSinkListener sinkListener;
	IPdraw::IVipcSource *source = nullptr;
	IPdraw::IRawVideoSink *sink = nullptr;
	setupVipcSourceWithSink(loop,
				session,
				mediaListener,
				sourceListener,
				sinkListener,
				sourceParams,
				&source,
				&sink);
	auto sinkOwner = std::unique_ptr<IPdraw::IRawVideoSink>(sink);
	auto sourceOwner = std::unique_ptr<IPdraw::IVipcSource>(source);

	int ret = source->play();
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	bool gotRunning = loop.pumpUntil(
		[&server]() { return server.runningRemoteCount() > 0; }, 15000);
	CU_ASSERT_TRUE_FATAL(gotRunning);

	/* Send frame at 1s (1000000000ns) */
	server.sendFrame(1000000000ULL);

	/* Wait for frames */
	bool gotFrames = loop.pumpUntil(
		[&sourceListener]() {
			return sourceListener.mFrameReadyCount.load() >= 1;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotFrames);

	struct mbuf_raw_video_frame *f1 = nullptr;
	struct mbuf_raw_video_frame *f2 = nullptr;
	ret = mbuf_raw_video_frame_queue_pop(sinkListener.mQueue, &f1);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(f1);

	ret = mbuf_raw_video_frame_queue_pop(sinkListener.mQueue, &f2);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(f2);

	struct vdef_raw_frame info1 = {}, info2 = {};
	mbuf_raw_video_frame_get_frame_info(f1, &info1);
	mbuf_raw_video_frame_get_frame_info(f2, &info2);

	/* f1 is the inserted grey frame (index 0), f2 is the real frame whose
	 * index was updated to 1 */
	CU_ASSERT_EQUAL(info1.info.index, 0);
	CU_ASSERT_EQUAL(info2.info.index, 1);

	mbuf_raw_video_frame_unref(f1);
	mbuf_raw_video_frame_unref(f2);

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);

	sinkOwner.reset();
	sourceOwner.reset();
}


static void testCxxVipcSourceDefaultBackendSelection()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	static const char *kVipcAddr = "unix:@pdraw_test_vipc_default_be";
	VipcTestServer server(loop.raw(), kVipcAddr);

	/* 1. backend_name = nullptr -> defaults to cBackends[0] */
	struct pdraw_vipc_source_params sourceParams1 = {};
	sourceParams1.address = kVipcAddr;
	sourceParams1.backend_name = nullptr;
	VipcSourceTrackingListener sourceListener1;
	IPdraw::IVipcSource *source1 = nullptr;
	int ret = session->createVipcSource(
		&sourceParams1, &sourceListener1, &source1);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	auto owner1 = std::unique_ptr<IPdraw::IVipcSource>(source1);

	bool gotReady1 = loop.pumpUntil(
		[&sourceListener1]() {
			return sourceListener1.mGotReadyToPlay;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotReady1);

	owner1.reset();

	/* 2. backend_name = "default" -> goto default_backend */
	struct pdraw_vipc_source_params sourceParams2 = {};
	sourceParams2.address = kVipcAddr;
	sourceParams2.backend_name = "default";
	VipcSourceTrackingListener sourceListener2;
	IPdraw::IVipcSource *source2 = nullptr;
	ret = session->createVipcSource(
		&sourceParams2, &sourceListener2, &source2);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	auto owner2 = std::unique_ptr<IPdraw::IVipcSource>(source2);

	bool gotReady2 = loop.pumpUntil(
		[&sourceListener2]() {
			return sourceListener2.mGotReadyToPlay;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotReady2);

	owner2.reset();

	/* 3. backend_name = "unknown_backend_xyz" -> ULOGW then falls back to
	 * default */
	struct pdraw_vipc_source_params sourceParams3 = {};
	sourceParams3.address = kVipcAddr;
	sourceParams3.backend_name = "unknown_backend_xyz";
	VipcSourceTrackingListener sourceListener3;
	IPdraw::IVipcSource *source3 = nullptr;
	ret = session->createVipcSource(
		&sourceParams3, &sourceListener3, &source3);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	auto owner3 = std::unique_ptr<IPdraw::IVipcSource>(source3);

	bool gotReady3 = loop.pumpUntil(
		[&sourceListener3]() {
			return sourceListener3.mGotReadyToPlay;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotReady3);

	owner3.reset();

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
}


static void testCxxVipcSourceWrapperGuardsAfterElementCleared()
{
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	static const char *kVipcAddr = "unix:@pdraw_test_vipc_guards";
	VipcTestServer server(loop.raw(), kVipcAddr);

	struct pdraw_vipc_source_params sourceParams = {};
	sourceParams.address = kVipcAddr;
	sourceParams.backend_name = "shm";
	VipcSourceTrackingListener sourceListener;
	QueueDrainingRawVideoSinkListener sinkListener;
	IPdraw::IVipcSource *source = nullptr;
	IPdraw::IRawVideoSink *sink = nullptr;
	setupVipcSourceWithSink(loop,
				session,
				mediaListener,
				sourceListener,
				sinkListener,
				sourceParams,
				&source,
				&sink);
	auto sourceOwner = std::unique_ptr<IPdraw::IVipcSource>(source);
	auto sinkOwner = std::unique_ptr<IPdraw::IRawVideoSink>(sink);

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
	CU_ASSERT_EQUAL(mediaListener.mStopStatus, 0);

	CU_ASSERT_PTR_NULL(
		static_cast<VipcSourceWrapper *>(source)->getVipcSource());

	struct vmeta_session meta = {};
	CU_ASSERT_FALSE(source->isReadyToPlay());
	CU_ASSERT_FALSE(source->isPaused());
	CU_ASSERT_EQUAL(source->play(), -EPROTO);
	CU_ASSERT_EQUAL(source->pause(), -EPROTO);
	CU_ASSERT_EQUAL(source->configure(nullptr, nullptr), -EPROTO);
	CU_ASSERT_EQUAL(source->insertGreyFrame(0), -EPROTO);
	CU_ASSERT_EQUAL(source->setSessionMetadata(&meta), -EPROTO);
	CU_ASSERT_EQUAL(source->getSessionMetadata(&meta), -EPROTO);

	sinkOwner.reset();
	sourceOwner.reset();
}

#endif /* PDRAW_TEST_VIPC_SOURCE_ENABLED */


CU_TestInfo g_pdraw_test_pipeline_vipc[] = {
#ifdef PDRAW_TEST_VIPC_SOURCE_ENABLED
	{FN("testCxxVipcSourceReceivesRealFrameFromShmServer"),
	 testCxxVipcSourceReceivesRealFrameFromShmServer},
	{FN("testCxxVipcSourcePauseDrainsAndCallsPauseResponse"),
	 testCxxVipcSourcePauseDrainsAndCallsPauseResponse},
	{FN("testCxxVipcSourceConfigureReturnsNotImplemented"),
	 testCxxVipcSourceConfigureReturnsNotImplemented},
	{FN("testCxxVipcSourceInsertGreyFrameProducesFakeFrame"),
	 testCxxVipcSourceInsertGreyFrameProducesFakeFrame},
	{FN("testCxxVipcSourceSessionMetadataRoundtrips"),
	 testCxxVipcSourceSessionMetadataRoundtrips},
	{FN("testCxxVipcSourceServerEosNotifiesListener"),
	 testCxxVipcSourceServerEosNotifiesListener},
	{FN("testCxxVipcSourceFrameWatchdogFiresOnTimeout"),
	 testCxxVipcSourceFrameWatchdogFiresOnTimeout},
	{FN("testCxxVipcSourceConnectionWatchdogFiresWhenNoServer"),
	 testCxxVipcSourceConnectionWatchdogFiresWhenNoServer},
	{FN("testCxxVipcSourceProcessFrameRejectsWrongFormat"),
	 testCxxVipcSourceProcessFrameRejectsWrongFormat},
	{FN("testCxxVipcSourceProcessFrameRejectsWrongResolution"),
	 testCxxVipcSourceProcessFrameRejectsWrongResolution},
	{FN("testCxxVipcSourceProcessFrameRejectsNonMonotonicTimestamp"),
	 testCxxVipcSourceProcessFrameRejectsNonMonotonicTimestamp},
	{FN("testCxxVipcSourceFramerateChangeRecreatesMedia"),
	 testCxxVipcSourceFramerateChangeRecreatesMedia},
	{FN("testCxxVipcSourceMemImplemGenericAcceptedForShmBackend"),
	 testCxxVipcSourceMemImplemGenericAcceptedForShmBackend},
	{FN("testCxxVipcSourceMemImplemMismatchRejectedByBackend"),
	 testCxxVipcSourceMemImplemMismatchRejectedByBackend},
	{FN("testCxxVipcSourceIdenticalStatusResendIgnored"),
	 testCxxVipcSourceIdenticalStatusResendIgnored},
	{FN("testCxxVipcSourceFramerateChangeIgnoredKeepsMedia"),
	 testCxxVipcSourceFramerateChangeIgnoredKeepsMedia},
	{FN("testCxxVipcSourceEosRestartReasonPropagates"),
	 testCxxVipcSourceEosRestartReasonPropagates},
	{FN("testCxxVipcSourceEosConfigurationReasonPropagates"),
	 testCxxVipcSourceEosConfigurationReasonPropagates},
	{FN("testCxxVipcSourceEosTriggeredFlushReachesSinkListener"),
	 testCxxVipcSourceEosTriggeredFlushReachesSinkListener},
	{FN("testCxxVipcSourceEosIgnoredNotifiesNotReady"),
	 testCxxVipcSourceEosIgnoredNotifiesNotReady},
	{FN("testCxxVipcSourceIdenticalStatusResendWhileRunningRearmsWatchdog"),
	 testCxxVipcSourceIdenticalStatusResendWhileRunningRearmsWatchdog},
	{FN("testCxxVipcSourceCreateMediaRestartsRunningAfterIgnoredEos"),
	 testCxxVipcSourceCreateMediaRestartsRunningAfterIgnoredEos},
	{FN("testCVipcSourceListenerCbsWithShmServer"),
	 testCVipcSourceListenerCbsWithShmServer},
	{FN("testCxxVipcSourcePlayMultipleTimesNoop"),
	 testCxxVipcSourcePlayMultipleTimesNoop},
	{FN("testCxxVipcSourcePlayWhilePausePendingReturnsEbusy"),
	 testCxxVipcSourcePlayWhilePausePendingReturnsEbusy},
	{FN("testCxxVipcSourcePlayWhenNotReadyReturnsEproto"),
	 testCxxVipcSourcePlayWhenNotReadyReturnsEproto},
	{FN("testCxxVipcSourceInsertGreyFrameRaw16"),
	 testCxxVipcSourceInsertGreyFrameRaw16},
	{FN("testCxxVipcSourceInsertGreyFrameUnsupportedFormat"),
	 testCxxVipcSourceInsertGreyFrameUnsupportedFormat},
	{FN("testCxxVipcSourceMaxPushedFrameCountAutoPauses"),
	 testCxxVipcSourceMaxPushedFrameCountAutoPauses},
	{FN("testCxxVipcSourceCustomTimescale"),
	 testCxxVipcSourceCustomTimescale},
	{FN("testCxxVipcSourceSmallTimescaleScaledUp"),
	 testCxxVipcSourceSmallTimescaleScaledUp},
	{FN("testCxxVipcSourceDecimation"), testCxxVipcSourceDecimation},
	{FN("testCxxVipcSourceFriendlyName"), testCxxVipcSourceFriendlyName},
	{FN("testCxxVipcSourceCustomCrop"), testCxxVipcSourceCustomCrop},
	{FN("testCxxVipcSourceInsertGreyFrameNullStatusReturnsEproto"),
	 testCxxVipcSourceInsertGreyFrameNullStatusReturnsEproto},
	{FN("testCxxVipcSourceStopWithInFlightFrame"),
	 testCxxVipcSourceStopWithInFlightFrame},
	{FN("testCxxVipcSourceProcessFrameRejectsWrongFullRange"),
	 testCxxVipcSourceProcessFrameRejectsWrongFullRange},
	{FN("testCxxVipcSourceInsertGreyFrameInFrameReadyUpdatesIndex"),
	 testCxxVipcSourceInsertGreyFrameInFrameReadyUpdatesIndex},
	{FN("testCxxVipcSourceDefaultBackendSelection"),
	 testCxxVipcSourceDefaultBackendSelection},
	{FN("testCxxVipcSourceWrapperGuardsAfterElementCleared"),
	 testCxxVipcSourceWrapperGuardsAfterElementCleared},
#endif /* PDRAW_TEST_VIPC_SOURCE_ENABLED */
	CU_TEST_INFO_NULL,
};
