/**
 * Parrot Drones Audio and Video Vector library
 * libpdraw unit tests — audio source -> sink roundtrip on real WAV (PCM)
 * and ADTS (AAC-LC) files (Tier B, self-contained fixture)
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

/* Adapted from libpdraw-backend/tests/pdraw_audiosourcesink_test.c.
 *
 * Unlike video, pdraw has a single IAudioSource/IAudioSink pair for BOTH
 * raw PCM and coded AAC-LC audio: struct adef_format carries an `encoding`
 * (ADEF_ENCODING_PCM or ADEF_ENCODING_AAC_LC) and ExternalAudioSource/
 * ExternalAudioSink are format-agnostic (see getSupportedFormats() in
 * pdraw_external_audio_sink.cpp, which explicitly lists both PCM and
 * AAC-LC/ADTS variants). So this one file covers both:
 *  - *.wav (PCM): read/written via libaudio-raw's araw_reader/araw_writer,
 *    exactly like the reference program. WAV headers are self-describing,
 *    so no format needs to be hardcoded (mirrors *.y4m for video).
 *  - *.aac (AAC-LC/ADTS elementary stream): no araw_reader support (it only
 *    knows PCM/WAV); parsed by hand with libaac's aac_reader, mirroring how
 *    the coded video test parses NALUs with h264_reader/h265_reader. Each
 *    ADTS frame (header + payload, aac_frame_length bytes) is treated as one
 *    atomic mbuf_audio_frame buffer -- aac_adts_to_adef_format() derives the
 *    adef_format straight from the first frame's ADTS header (channel
 *    count, sample rate; AAC has no embedded bit depth, hardcoded to 16 by
 *    that conversion). No out-of-band AudioSpecificConfig is set: pdraw's
 *    setAacAsc() is only ever called by the MP4 demuxer/encoder paths, never
 *    by ExternalAudioSource, so it plays no part in a source/sink roundtrip.
 *
 * Same two-phase structure as the coded/raw video tests: collect every
 * frame from the input file first (independently of pdraw), then only once
 * a sink exists on the source's media, push every frame and drain the
 * sink's queue -- pushing into the source before a sink is connected would
 * let ExternalAudioSource::process() silently drop the frames.
 *
 * Byte-identical roundtrip risk (WAV only): unlike the AAC/video cases,
 * *.wav is a RIFF container, and araw_writer_new() synthesizes a fresh
 * minimal header (fmt + data chunks) from the resolved adef_format rather
 * than copying the original one verbatim. If the source file carries extra
 * chunks (LIST/INFO/fact...) or a data chunk padded to an even size, the
 * sample DATA would still roundtrip exactly but the raw file bytes could
 * differ. Tried as a plain whole-file byte diff first, per the same
 * risk-vs-simplicity tradeoff already made for the coded video test; if
 * that turns out to fail on these specific assets, fall back to comparing
 * only the `data` chunk's payload bytes instead of the whole file. */

#define ULOG_TAG pdraw_test_pipeline_sourcesink_audio
#include "test_api_common.hpp"
#include "test_common.h"
#include "test_fixtures.hpp"
#include "test_pipeline_common.hpp"

#include "pdraw_external_audio_sink.hpp"

#include <aac/aac.h>
#include <audio-defs/adefs.h>
#include <audio-raw/araw.h>
#include <media-buffers/mbuf_audio_frame.h>
#include <media-buffers/mbuf_mem_generic.h>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <vector>

ULOG_DECLARE_TAG(ULOG_TAG);

using namespace Pdraw;
using namespace PdrawTest;


/* ── Audio elementary stream fixtures ────────────────────────────────────
 * (NAS assets, same ASSETS_ROOT/PDRAW_GET_ASSET_PATH mechanism as every
 * other test_pipeline_*.cpp file). */

enum {
	ASSET_WAV_MONO = 0,
	ASSET_WAV_STEREO = 1,
	ASSET_AAC_MONO = 2,
	ASSET_AAC_STEREO = 3,
};

static constexpr struct {
	const char *relative_path;
} s_assets_sourcesink_audio[] = {
	{"Tests/miscellaneous/song_mono.wav"},
	{"Tests/miscellaneous/song_stereo.wav"},
	{"Tests/miscellaneous/song_mono.aac"},
	{"Tests/miscellaneous/song_stereo.aac"},
};

/* Number of samples per AAC-LC frame (fixed by the codec, not stored in
 * the ADTS header). Used only to build strictly-increasing per-frame
 * timestamps (see ExternalAudioSource::inputFilter()'s requirement). */
#define AAC_LC_SAMPLES_PER_FRAME 1024


/* Session-wide listener: correlates onMediaAdded() with our own source
 * instance via elementUserData, same rationale as the video tests'
 * SourceMediaListener. */
/* Anonymous namespace: internal linkage for the listener/helper classes
 * below, to avoid ODR violations with identically-named classes defined
 * differently in sibling test_*.cpp files linked into the same binary
 * (see test_pipeline_vipc_source.cpp for the bug this pattern was found to
 * fix). */
namespace {

class AudioSourceMediaListener : public IPdraw::Listener {
public:
	void stopResponse(IPdraw * /*p*/, int status) override
	{
		mStopStatus = status;
		mGotStopResponse = true;
	}

	void onMediaAdded(IPdraw * /*p*/,
			  const struct pdraw_media_info *info,
			  void *elementUserData) override
	{
		if (elementUserData != mSource)
			return;
		mMediaId = info->id;
		mGotMediaAdded = true;
	}

	void onMediaRemoved(IPdraw * /*p*/,
			    const struct pdraw_media_info * /*i*/,
			    void * /*u*/) override
	{
	}

	void onSocketCreated(IPdraw * /*p*/, int /*fd*/) override {}

	void *mSource = nullptr;
	bool mGotMediaAdded = false;
	unsigned int mMediaId = 0;
	bool mGotStopResponse = false;
	int mStopStatus = 0;
};


/* Tracks IAudioSource::flush()/drain() completion. Same rationale as the
 * video tests' DrainTrackingXxxSourceListener: used only by the happy-path
 * roundtrip test below, which calls drain() before tearing down the source,
 * matching the libpdraw-backend reference programs' lifecycle. */
class DrainTrackingAudioSourceListener : public IPdraw::IAudioSource::Listener {
public:
	void onAudioSourceFlushed(IPdraw * /*p*/,
				  IPdraw::IAudioSource * /*src*/) override
	{
		mGotFlushed = true;
	}

	void onAudioSourceDrained(IPdraw * /*p*/,
				  IPdraw::IAudioSource * /*src*/) override
	{
		mGotDrained = true;
	}

	bool mGotFlushed = false;
	bool mGotDrained = false;
};


/* Implements the sink side of the flush/drain protocol for real: pop (and
 * unref) every frame already sitting in the sink's own queue, then
 * acknowledge via queueFlushed()/queueDrained(). Without this, calling
 * IAudioSource::drain() against g_stub_audio_sink_listener (a no-op stub,
 * used by every other test in this file) would hang forever -- the stub
 * never acknowledges, so Channel::drainDone() never fires and
 * onAudioSourceDrained() never fires either. */
class QueueDrainingAudioSinkListener : public IPdraw::IAudioSink::Listener {
public:
	void
	onAudioSinkMediaAdded(IPdraw * /*p*/,
			      IPdraw::IAudioSink * /*sk*/,
			      const struct pdraw_media_info * /*i*/) override
	{
	}

	void onAudioSinkMediaRemoved(IPdraw * /*p*/,
				     IPdraw::IAudioSink * /*sk*/,
				     const struct pdraw_media_info * /*i*/,
				     bool /*restart*/) override
	{
	}

	void onAudioSinkFlush(IPdraw * /*p*/, IPdraw::IAudioSink *sk) override
	{
		/* Flush really means discard: unref outright. */
		discardQueue();
		sk->queueFlushed();
	}

	void onAudioSinkDrain(IPdraw * /*p*/, IPdraw::IAudioSink *sk) override
	{
		/* Unlike flush, drain must not lose data: whatever is in the
		 * queue at this point may be real frames process() just
		 * forwarded (see ExternalAudioSource::flush(discard=false),
		 * which calls process() BEFORE draining the output channel)
		 * -- accumulate into mDrainedFrames instead of unreffing
		 * outright, so a test can inspect what was delivered. The
		 * caller owns the refs afterward. */
		while (true) {
			struct mbuf_audio_frame *f = nullptr;
			if (mQueue == nullptr ||
			    mbuf_audio_frame_queue_pop(mQueue, &f) != 0)
				break;
			mDrainedFrames.push_back(f);
		}
		sk->queueDrained();
	}

	/* Must be set right after createAudioSink() returns, before any
	 * flush/drain can occur. */
	struct mbuf_audio_frame_queue *mQueue = nullptr;

	/* Populated by onAudioSinkDrain(); empty in the roundtrip tests
	 * (outQueue is already drained by the test itself before drain() is
	 * ever called there), populated for real in
	 * testCxxAudioSourceDrainForwardsUnprocessedFrames. Caller must unref
	 * each entry. */
	std::vector<struct mbuf_audio_frame *> mDrainedFrames;

private:
	void discardQueue()
	{
		if (mQueue == nullptr)
			return;
		struct mbuf_audio_frame *f = nullptr;
		while (mbuf_audio_frame_queue_pop(mQueue, &f) == 0)
			mbuf_audio_frame_unref(f);
	}
};


/* Acknowledges flush/drain with the WRONG companion function on purpose, to
 * exercise ExternalAudioSink::flushDone()'s mFlushDiscard mismatch warning
 * path (pdraw_external_audio_sink.cpp:282-287) via
 * testCxxAudioSinkFlushAckMismatchFollowsRequestedDiscardState below. Only
 * onAudioSinkFlush() needs to misbehave for that test; onAudioSinkDrain()
 * still acks correctly since it is not exercised there. */
class MismatchAckAudioSinkListener : public IPdraw::IAudioSink::Listener {
public:
	void
	onAudioSinkMediaAdded(IPdraw * /*p*/,
			      IPdraw::IAudioSink * /*sk*/,
			      const struct pdraw_media_info * /*i*/) override
	{
		mGotMediaAdded = true;
	}

	void onAudioSinkMediaRemoved(IPdraw * /*p*/,
				     IPdraw::IAudioSink * /*sk*/,
				     const struct pdraw_media_info * /*i*/,
				     bool /*restart*/) override
	{
	}

	void onAudioSinkFlush(IPdraw * /*p*/, IPdraw::IAudioSink *sk) override
	{
		/* Wrong ack on purpose: a real flush(discard=true) is
		 * underway (mFlushDiscard==true), but acknowledge as if it
		 * were a drain. */
		sk->queueDrained();
	}

	void onAudioSinkDrain(IPdraw * /*p*/, IPdraw::IAudioSink *sk) override
	{
		sk->queueDrained();
	}

	bool mGotMediaAdded = false;
};


/* Tracks IAudioSink media-added/media-removed callbacks; used only by
 * testCxxAudioSourceSinkSwitchMediaId below to observe setMediaId()'s
 * effects. Flush/drain are left as no-ops, like g_stub_audio_sink_listener
 * (shared by most other tests in this file): that test never calls
 * flush()/drain()/resync() itself, so no acknowledgement is required. */
class AudioSinkTrackingListener : public IPdraw::IAudioSink::Listener {
public:
	void
	onAudioSinkMediaAdded(IPdraw * /*p*/,
			      IPdraw::IAudioSink * /*sk*/,
			      const struct pdraw_media_info * /*i*/) override
	{
		mGotMediaAdded = true;
	}

	void onAudioSinkMediaRemoved(IPdraw * /*p*/,
				     IPdraw::IAudioSink * /*sk*/,
				     const struct pdraw_media_info * /*i*/,
				     bool restart) override
	{
		mGotMediaRemoved = true;
		mLastRemovedRestart = restart;
	}

	void onAudioSinkFlush(IPdraw * /*p*/,
			      IPdraw::IAudioSink * /*sk*/) override
	{
	}

	void onAudioSinkDrain(IPdraw * /*p*/,
			      IPdraw::IAudioSink * /*sk*/) override
	{
	}

	bool mGotMediaAdded = false;
	bool mGotMediaRemoved = false;
	bool mLastRemovedRestart = false;
};


/* ── File comparison helper (same as the video tests) ────────────────────── */

static bool filesAreIdentical(const char *pathA, const char *pathB)
{
	struct stat stA = {};
	struct stat stB = {};
	if (stat(pathA, &stA) != 0 || stat(pathB, &stB) != 0)
		return false;
	if (stA.st_size != stB.st_size)
		return false;

	FILE *fa = fopen(pathA, "rb");
	FILE *fb = fopen(pathB, "rb");
	if (fa == nullptr || fb == nullptr) {
		if (fa != nullptr)
			fclose(fa);
		if (fb != nullptr)
			fclose(fb);
		return false;
	}

	bool identical = true;
	uint8_t bufA[4096];
	uint8_t bufB[4096];
	while (true) {
		size_t ra = fread(bufA, 1, sizeof(bufA), fa);
		size_t rb = fread(bufB, 1, sizeof(bufB), fb);
		if (ra != rb || memcmp(bufA, bufB, ra) != 0) {
			identical = false;
			break;
		}
		if (ra == 0)
			break;
	}
	fclose(fa);
	fclose(fb);
	return identical;
}


/* ── Phase 1a: collect frames from a *.wav (PCM) file via araw_reader ───── */

static void collectWavFrames(const char *inPath,
			     struct adef_format *format,
			     std::vector<struct mbuf_audio_frame *> *frames,
			     std::vector<struct mbuf_mem *> *mems)
{
	struct araw_reader_config readerConfig = {};
	struct araw_reader *reader = nullptr;
	int ret = araw_reader_new(inPath, &readerConfig, &reader);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	ret = araw_reader_get_config(reader, &readerConfig);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	*format = readerConfig.format;

	size_t frameLen = 1024 * readerConfig.format.channel_count *
			  (readerConfig.format.bit_depth / 8);
	CU_ASSERT_FATAL(frameLen > 0);

	while (true) {
		struct mbuf_mem *mem = nullptr;
		ret = mbuf_mem_generic_new(frameLen, &mem);
		CU_ASSERT_EQUAL_FATAL(ret, 0);

		uint8_t *data = nullptr;
		size_t capacity = 0;
		ret = mbuf_mem_get_data(mem, (void **)&data, &capacity);
		CU_ASSERT_EQUAL_FATAL(ret, 0);

		struct araw_frame inFrame = {};
		ret = araw_reader_frame_read(reader, data, capacity, &inFrame);
		if (ret == -ENOENT) {
			mbuf_mem_unref(mem);
			break;
		}
		CU_ASSERT_EQUAL_FATAL(ret, 0);

		struct mbuf_audio_frame *frame = nullptr;
		ret = mbuf_audio_frame_new(&inFrame.frame, &frame);
		CU_ASSERT_EQUAL_FATAL(ret, 0);

		ret = mbuf_audio_frame_set_buffer(frame, mem, 0, capacity);
		CU_ASSERT_EQUAL_FATAL(ret, 0);

		ret = mbuf_audio_frame_finalize(frame);
		CU_ASSERT_EQUAL_FATAL(ret, 0);

		frames->push_back(frame);
		mems->push_back(mem);
	}
	araw_reader_destroy(reader);

	CU_ASSERT_FALSE_FATAL(frames->empty());
}


/* ── Phase 1b: collect frames from a *.aac (ADTS/AAC-LC) file by hand ────── */

struct AacParseContext {
	struct adef_format format = {};
	bool configured = false;
	uint32_t index = 0;
	uint64_t timestamp = 0;
	std::vector<struct mbuf_audio_frame *> *frames = nullptr;
	std::vector<struct mbuf_mem *> *mems = nullptr;
};

static void aac_adts_frame_end_cb(struct aac_ctx * /*ctx*/,
				  const uint8_t *buf,
				  size_t len,
				  const struct aac_adts *adts,
				  void *userdata)
{
	auto *pctx = static_cast<struct AacParseContext *>(userdata);

	if (!pctx->configured) {
		int ret = aac_adts_to_adef_format(adts, &pctx->format);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
		pctx->configured = true;
	}

	struct mbuf_mem *mem = nullptr;
	int ret = mbuf_mem_generic_new(len, &mem);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	uint8_t *data = nullptr;
	size_t capacity = 0;
	ret = mbuf_mem_get_data(mem, (void **)&data, &capacity);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_FATAL(capacity >= len);
	memcpy(data, buf, len);

	struct adef_frame frameInfo = {};
	frameInfo.format = pctx->format;
	frameInfo.info.timescale = pctx->format.sample_rate;
	frameInfo.info.timestamp = pctx->timestamp;
	frameInfo.info.index = pctx->index;

	struct mbuf_audio_frame *frame = nullptr;
	ret = mbuf_audio_frame_new(&frameInfo, &frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	ret = mbuf_audio_frame_set_buffer(frame, mem, 0, len);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	ret = mbuf_audio_frame_finalize(frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	pctx->frames->push_back(frame);
	pctx->mems->push_back(mem);
	pctx->index++;
	pctx->timestamp += AAC_LC_SAMPLES_PER_FRAME;
}


static const struct aac_ctx_cbs kAacCbs = [] {
	struct aac_ctx_cbs cbs = {};
	cbs.adts_frame_end = aac_adts_frame_end_cb;
	return cbs;
}();

struct MappedFile {
	int fd = -1;
	void *data = nullptr;
	size_t len = 0;

	~MappedFile()
	{
		if (data != nullptr && len > 0)
			munmap(data, len);
		if (fd >= 0)
			close(fd);
	}
};

static bool mapFile(const char *path, struct MappedFile *out)
{
	out->fd = open(path, O_RDONLY);
	if (out->fd < 0)
		return false;
	off_t size = lseek(out->fd, 0, SEEK_END);
	if (size < 0)
		return false;
	out->len = static_cast<size_t>(size);
	out->data = mmap(nullptr, out->len, PROT_READ, MAP_PRIVATE, out->fd, 0);
	return out->data != MAP_FAILED;
}


/* Fallback for the WAV roundtrip check (see the file-level comment): locate
 * the "data" chunk of a RIFF/WAVE file and return its offset/size, walking
 * chunks by their declared size rather than assuming a fixed header layout
 * so any extra chunks (LIST/INFO/fact...) in the original file are simply
 * skipped over. */
static bool findWavDataChunk(const uint8_t *buf,
			     size_t len,
			     size_t *dataOffset,
			     size_t *dataSize)
{
	if (len < 12 || memcmp(buf, "RIFF", 4) != 0 ||
	    memcmp(buf + 8, "WAVE", 4) != 0)
		return false;

	size_t off = 12;
	while (off + 8 <= len) {
		const uint8_t *id = buf + off;
		uint32_t chunkSize =
			static_cast<uint32_t>(buf[off + 4]) |
			(static_cast<uint32_t>(buf[off + 5]) << 8) |
			(static_cast<uint32_t>(buf[off + 6]) << 16) |
			(static_cast<uint32_t>(buf[off + 7]) << 24);
		size_t chunkDataOff = off + 8;
		if (memcmp(id, "data", 4) == 0) {
			if (chunkDataOff + chunkSize > len)
				return false;
			*dataOffset = chunkDataOff;
			*dataSize = chunkSize;
			return true;
		}
		/* Chunks are word-aligned: a pad byte follows odd-sized data.
		 */
		off = chunkDataOff + chunkSize + (chunkSize & 1);
	}
	return false;
}

/* araw_reader_frame_read() reads fixed-size (1024-sample) blocks and
 * returns -ENOENT as soon as fewer than a full block remains (see
 * araw_reader.c: "if ((size_t)ret != len) return -ENOENT;"): any trailing
 * partial block (up to 1023 samples) is silently dropped by the reader
 * itself, independently of pdraw. So the output "data" chunk is expected
 * to be a whole number of 1024-sample blocks, generally a bit shorter than
 * the input's -- compare only the common prefix rather than requiring
 * equal sizes, and require the output to not be shorter than that (which
 * would indicate an actual roundtrip bug, not just the known tail drop). */
static bool wavDataChunksAreIdentical(const char *pathA, const char *pathB)
{
	struct MappedFile a;
	struct MappedFile b;
	if (!mapFile(pathA, &a) || !mapFile(pathB, &b))
		return false;

	size_t offA = 0;
	size_t sizeA = 0;
	size_t offB = 0;
	size_t sizeB = 0;
	if (!findWavDataChunk(
		    static_cast<const uint8_t *>(a.data), a.len, &offA, &sizeA))
		return false;
	if (!findWavDataChunk(
		    static_cast<const uint8_t *>(b.data), b.len, &offB, &sizeB))
		return false;
	if (sizeB > sizeA)
		return false;

	return memcmp(static_cast<const uint8_t *>(a.data) + offA,
		      static_cast<const uint8_t *>(b.data) + offB,
		      sizeB) == 0;
}


static void collectAacFrames(const char *inPath,
			     struct adef_format *format,
			     std::vector<struct mbuf_audio_frame *> *frames,
			     std::vector<struct mbuf_mem *> *mems)
{
	struct MappedFile input;
	CU_ASSERT_TRUE_FATAL(mapFile(inPath, &input));

	struct AacParseContext ctx;
	ctx.frames = frames;
	ctx.mems = mems;

	struct aac_reader *reader = nullptr;
	int ret = aac_reader_new(&kAacCbs, &ctx, &reader);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	size_t inOff = 0;
	while (inOff < input.len) {
		size_t off = 0;
		const uint8_t *buf =
			static_cast<const uint8_t *>(input.data) + inOff;
		ret = aac_reader_parse(reader, 0, buf, input.len - inOff, &off);
		CU_ASSERT_FATAL(ret >= 0);
		inOff += off;
		if (off == 0)
			break; /* no progress: avoid an infinite loop */
	}
	aac_reader_stop(reader);
	aac_reader_destroy(reader);

	CU_ASSERT_FALSE_FATAL(frames->empty());
	*format = ctx.format;
}


/* ── The test itself ──────────────────────────────────────────────────── */

enum AudioAssetKind { AUDIO_KIND_WAV, AUDIO_KIND_AAC };

static void runAudioSourceSinkRoundtrip(enum AudioAssetKind kind,
					size_t assetIndex,
					const char *outSuffix)
{
	char inPath[512];
	PDRAW_GET_ASSET_PATH(inPath, assetIndex, s_assets_sourcesink_audio);

	struct adef_format format = {};
	std::vector<struct mbuf_audio_frame *> inputFrames;
	std::vector<struct mbuf_mem *> inputMems;

	switch (kind) {
	case AUDIO_KIND_WAV:
		collectWavFrames(inPath, &format, &inputFrames, &inputMems);
		break;
	case AUDIO_KIND_AAC:
		collectAacFrames(inPath, &format, &inputFrames, &inputMems);
		break;
	}

	/* ── Real pdraw pipeline: source -> sink, same session ── */

	TestPompLoop loop;
	AudioSourceMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	struct pdraw_audio_source_params sourceParams = {};
	sourceParams.playback_type = PDRAW_PLAYBACK_TYPE_REPLAY;
	sourceParams.audio.format = format;

	DrainTrackingAudioSourceListener sourceListener;
	IPdraw::IAudioSource *source = nullptr;
	int ret = session->createAudioSource(
		&sourceParams, &sourceListener, &source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source);
	auto sourceOwner = std::unique_ptr<IPdraw::IAudioSource>(source);
	mediaListener.mSource = source;

	/* Pump now, before pushing any frame: see the video tests for why
	 * the sink must exist before any frame is pushed. */
	bool gotMediaAdded = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotMediaAdded; });
	CU_ASSERT_TRUE_FATAL(gotMediaAdded);
	CU_ASSERT_NOT_EQUAL_FATAL(mediaListener.mMediaId, 0u);

	QueueDrainingAudioSinkListener sinkListener;
	IPdraw::IAudioSink *sink = nullptr;
	ret = session->createAudioSink(
		mediaListener.mMediaId, &sinkListener, &sink);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(sink);
	auto sinkOwner = std::unique_ptr<IPdraw::IAudioSink>(sink);

	struct mbuf_audio_frame_queue *inQueue = source->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(inQueue);
	struct mbuf_audio_frame_queue *outQueue = sink->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(outQueue);
	sinkListener.mQueue = outQueue;

	/* Now that the sink is connected, push every collected frame. */
	for (auto *frame : inputFrames) {
		ret = mbuf_audio_frame_queue_push(inQueue, frame);
		CU_ASSERT_EQUAL(ret, 0);
	}

	std::vector<struct mbuf_audio_frame *> outputFrames;
	bool gotAllFrames = loop.pumpUntil(
		[&]() {
			struct mbuf_audio_frame *f = nullptr;
			while (mbuf_audio_frame_queue_pop(outQueue, &f) == 0)
				outputFrames.push_back(f);
			return outputFrames.size() >= inputFrames.size();
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotAllFrames);
	CU_ASSERT_EQUAL(outputFrames.size(), inputFrames.size());

	/* Write the output file. WAV needs a proper header (araw_writer);
	 * AAC is a plain concatenation of complete ADTS frames, exactly
	 * like the input. */
	char outPath[512];
	snprintf(outPath,
		 sizeof(outPath),
		 "/tmp/pdraw_test_pipeline_sourcesink%s",
		 outSuffix);

	struct araw_writer *writer = nullptr;
	FILE *outFile = nullptr;
	if (kind == AUDIO_KIND_WAV) {
		struct araw_writer_config writerConfig = {};
		writerConfig.format = format;
		ret = araw_writer_new(outPath, &writerConfig, &writer);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
	} else {
		outFile = fopen(outPath, "wb");
		CU_ASSERT_PTR_NOT_NULL_FATAL(outFile);
	}

	for (auto *frame : outputFrames) {
		const void *data = nullptr;
		size_t len = 0;
		ret = mbuf_audio_frame_get_buffer(frame, &data, &len);
		CU_ASSERT_EQUAL(ret, 0);

		if (kind == AUDIO_KIND_WAV) {
			struct adef_frame frameInfo = {};
			ret = mbuf_audio_frame_get_frame_info(frame,
							      &frameInfo);
			CU_ASSERT_EQUAL(ret, 0);

			struct araw_frame outFrame = {};
			outFrame.frame = frameInfo;
			outFrame.cdata = static_cast<const uint8_t *>(data);
			outFrame.cdata_length = len;
			ret = araw_writer_frame_write(writer, &outFrame);
			CU_ASSERT_EQUAL(ret, 0);
		} else {
			size_t written = fwrite(data, len, 1, outFile);
			CU_ASSERT_EQUAL(written, 1u);
		}

		ret = mbuf_audio_frame_release_buffer(frame, data);
		CU_ASSERT_EQUAL(ret, 0);
		mbuf_audio_frame_unref(frame);
	}

	if (writer != nullptr)
		araw_writer_destroy(writer);
	if (outFile != nullptr)
		fclose(outFile);

	/* The whole point: the roundtrip reproduces the input exactly.
	 * WAV: compare only the "data" chunk (see the file-level comment --
	 * araw_writer_new() synthesizes a fresh minimal RIFF header, so a
	 * whole-file diff can fail on header differences alone even when
	 * every sample is identical). AAC has no container to reconstruct:
	 * a plain whole-file diff is the exact right check. */
	if (kind == AUDIO_KIND_WAV) {
		CU_ASSERT_TRUE(wavDataChunksAreIdentical(inPath, outPath));
	} else {
		CU_ASSERT_TRUE(filesAreIdentical(inPath, outPath));
	}
	(void)remove(outPath);

	for (auto *frame : inputFrames)
		mbuf_audio_frame_unref(frame);
	for (auto *mem : inputMems)
		mbuf_mem_unref(mem);

	/* Drain, THEN flush, the source before tearing down -- in that order,
	 * deliberately: ExternalAudioSource::flush(discard=false, i.e.
	 * drain()) calls process() first, forcing anything still sitting in
	 * the source's own input queue (inQueue) to be forwarded downstream
	 * before completing, while flush(discard=true) skips that and
	 * discards whatever is left in inQueue outright. Draining first is
	 * what actually guarantees nothing is lost; flush() afterward is
	 * then safe by construction, not because this test happens to have
	 * already popped everything from outQueue above. */
	ret = source->drain();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotDrained = loop.pumpUntil(
		[&sourceListener]() { return sourceListener.mGotDrained; },
		15000);
	CU_ASSERT_TRUE(gotDrained);

	ret = source->flush();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotFlushed = loop.pumpUntil(
		[&sourceListener]() { return sourceListener.mGotFlushed; },
		15000);
	CU_ASSERT_TRUE(gotFlushed);

	sinkOwner.reset();
	sourceOwner.reset();

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
	CU_ASSERT_EQUAL(mediaListener.mStopStatus, 0);
}


static void testCxxAudioSourceSinkRoundtripWavMono()
{
	runAudioSourceSinkRoundtrip(
		AUDIO_KIND_WAV, ASSET_WAV_MONO, ".mono.wav");
}


static void testCxxAudioSourceSinkRoundtripWavStereo()
{
	runAudioSourceSinkRoundtrip(
		AUDIO_KIND_WAV, ASSET_WAV_STEREO, ".stereo.wav");
}


static void testCxxAudioSourceSinkRoundtripAacMono()
{
	runAudioSourceSinkRoundtrip(
		AUDIO_KIND_AAC, ASSET_AAC_MONO, ".mono.aac");
}


static void testCxxAudioSourceSinkRoundtripAacStereo()
{
	runAudioSourceSinkRoundtrip(
		AUDIO_KIND_AAC, ASSET_AAC_STEREO, ".stereo.aac");
}


/* ── Negative / edge-case tests: exercise ExternalAudioSource::
 * inputFilter() (format intersect check, strictly-increasing timestamp
 * check) -- mirrors the video tests' negative tests (see there for the
 * ref-counting rationale, identical here). No queue_max_count test: unlike
 * pdraw_video_source_params, pdraw_audio_source_params has no such field
 * (the audio source's input queue is always unbounded). ────────────────── */

static void
createAudioSourceAndSink(IPdraw *session,
			 TestPompLoop *loop,
			 AudioSourceMediaListener *mediaListener,
			 const struct pdraw_audio_source_params *sourceParams,
			 IPdraw::IAudioSource **source,
			 IPdraw::IAudioSink **sink,
			 struct mbuf_audio_frame_queue **inQueue,
			 struct mbuf_audio_frame_queue **outQueue)
{
	int ret = session->createAudioSource(
		sourceParams, &g_stub_audio_source_listener, source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(*source);
	mediaListener->mSource = *source;

	bool gotMediaAdded = loop->pumpUntil(
		[mediaListener]() { return mediaListener->mGotMediaAdded; });
	CU_ASSERT_TRUE_FATAL(gotMediaAdded);
	CU_ASSERT_NOT_EQUAL_FATAL(mediaListener->mMediaId, 0u);

	ret = session->createAudioSink(
		mediaListener->mMediaId, &g_stub_audio_sink_listener, sink);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(*sink);

	*inQueue = (*source)->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(*inQueue);
	*outQueue = (*sink)->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(*outQueue);
}

/* A minimal, validly-finalized frame with placeholder buffer content --
 * content is irrelevant, these tests only care about acceptance/rejection
 * at the queue's filter. */
static struct mbuf_audio_frame *
makeDummyAudioFrame(const struct adef_format &format, uint64_t timestamp)
{
	struct adef_frame frameInfo = {};
	frameInfo.format = format;
	frameInfo.info.timescale = format.sample_rate;
	frameInfo.info.timestamp = timestamp;

	struct mbuf_audio_frame *frame = nullptr;
	int ret = mbuf_audio_frame_new(&frameInfo, &frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	const size_t bufLen = 16;
	struct mbuf_mem *mem = nullptr;
	ret = mbuf_mem_generic_new(bufLen, &mem);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	ret = mbuf_audio_frame_set_buffer(frame, mem, 0, bufLen);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	mbuf_mem_unref(mem); /* the frame holds its own ref via set_buffer */

	ret = mbuf_audio_frame_finalize(frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	return frame;
}


static void testCxxAudioSourceRejectsFormatMismatch()
{
	char inPath[512];
	PDRAW_GET_ASSET_PATH(inPath, ASSET_WAV_MONO, s_assets_sourcesink_audio);

	struct adef_format format = {};
	std::vector<struct mbuf_audio_frame *> inputFrames;
	std::vector<struct mbuf_mem *> inputMems;
	collectWavFrames(inPath, &format, &inputFrames, &inputMems);
	CU_ASSERT_FATAL(inputFrames.size() >= 1);

	TestPompLoop loop;
	AudioSourceMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	struct pdraw_audio_source_params sourceParams = {};
	sourceParams.playback_type = PDRAW_PLAYBACK_TYPE_REPLAY;
	sourceParams.audio.format = format;

	IPdraw::IAudioSource *source = nullptr;
	IPdraw::IAudioSink *sink = nullptr;
	struct mbuf_audio_frame_queue *inQueue = nullptr;
	struct mbuf_audio_frame_queue *outQueue = nullptr;
	createAudioSourceAndSink(session,
				 &loop,
				 &mediaListener,
				 &sourceParams,
				 &source,
				 &sink,
				 &inQueue,
				 &outQueue);
	auto sourceOwner = std::unique_ptr<IPdraw::IAudioSource>(source);
	auto sinkOwner = std::unique_ptr<IPdraw::IAudioSink>(sink);

	/* A valid PCM frame is accepted... */
	int ret = mbuf_audio_frame_queue_push(inQueue, inputFrames[0]);
	CU_ASSERT_EQUAL(ret, 0);

	/* ...but an AAC-LC frame is rejected outright (the source was created
	 * for PCM), per ExternalAudioSource::inputFilter()'s
	 * adef_format_intersect() check -- never reaches the queue. */
	struct mbuf_audio_frame *badFrame = makeDummyAudioFrame(
		adef_aac_lc_16b_44100hz_mono_adts, UINT64_C(999999999));
	ret = mbuf_audio_frame_queue_push(inQueue, badFrame);
	CU_ASSERT_EQUAL(ret, -EPROTO);
	mbuf_audio_frame_unref(badFrame);

	/* The valid frame still made it through undisturbed. */
	std::vector<struct mbuf_audio_frame *> outputFrames;
	bool gotFrame = loop.pumpUntil(
		[&]() {
			struct mbuf_audio_frame *f = nullptr;
			while (mbuf_audio_frame_queue_pop(outQueue, &f) == 0)
				outputFrames.push_back(f);
			return !outputFrames.empty();
		},
		5000);
	CU_ASSERT_TRUE_FATAL(gotFrame);
	CU_ASSERT_EQUAL(outputFrames.size(), 1u);
	for (auto *f : outputFrames)
		mbuf_audio_frame_unref(f);

	for (auto *frame : inputFrames)
		mbuf_audio_frame_unref(frame);
	for (auto *mem : inputMems)
		mbuf_mem_unref(mem);

	sinkOwner.reset();
	sourceOwner.reset();
	stopSessionAndWait(&loop, session, &mediaListener);
}


static void testCxxAudioSourceRejectsNonMonotonicTimestamp()
{
	char inPath[512];
	PDRAW_GET_ASSET_PATH(inPath, ASSET_WAV_MONO, s_assets_sourcesink_audio);

	struct adef_format format = {};
	std::vector<struct mbuf_audio_frame *> inputFrames;
	std::vector<struct mbuf_mem *> inputMems;
	collectWavFrames(inPath, &format, &inputFrames, &inputMems);
	CU_ASSERT_FATAL(inputFrames.size() >= 2);

	TestPompLoop loop;
	AudioSourceMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	struct pdraw_audio_source_params sourceParams = {};
	sourceParams.playback_type = PDRAW_PLAYBACK_TYPE_REPLAY;
	sourceParams.audio.format = format;

	IPdraw::IAudioSource *source = nullptr;
	IPdraw::IAudioSink *sink = nullptr;
	struct mbuf_audio_frame_queue *inQueue = nullptr;
	struct mbuf_audio_frame_queue *outQueue = nullptr;
	createAudioSourceAndSink(session,
				 &loop,
				 &mediaListener,
				 &sourceParams,
				 &source,
				 &sink,
				 &inQueue,
				 &outQueue);
	auto sourceOwner = std::unique_ptr<IPdraw::IAudioSource>(source);
	auto sinkOwner = std::unique_ptr<IPdraw::IAudioSink>(sink);

	/* Push frame[1] (the later of the two by timestamp) first... */
	int ret = mbuf_audio_frame_queue_push(inQueue, inputFrames[1]);
	CU_ASSERT_EQUAL(ret, 0);

	/* ...then frame[0], whose earlier timestamp is now <= mLastTimestamp:
	 * rejected as non-strictly-monotonic. */
	ret = mbuf_audio_frame_queue_push(inQueue, inputFrames[0]);
	CU_ASSERT_EQUAL(ret, -EPROTO);

	std::vector<struct mbuf_audio_frame *> outputFrames;
	bool gotFrame = loop.pumpUntil(
		[&]() {
			struct mbuf_audio_frame *f = nullptr;
			while (mbuf_audio_frame_queue_pop(outQueue, &f) == 0)
				outputFrames.push_back(f);
			return !outputFrames.empty();
		},
		5000);
	CU_ASSERT_TRUE_FATAL(gotFrame);
	CU_ASSERT_EQUAL(outputFrames.size(), 1u);
	for (auto *f : outputFrames)
		mbuf_audio_frame_unref(f);

	for (auto *frame : inputFrames)
		mbuf_audio_frame_unref(frame);
	for (auto *mem : inputMems)
		mbuf_mem_unref(mem);

	sinkOwner.reset();
	sourceOwner.reset();
	stopSessionAndWait(&loop, session, &mediaListener);
}


/* ── flush() vs drain(): what happens to frames still queued in the
 * source's own input queue (inQueue) when neither has ever been forwarded
 * by a loop pump -- same rationale as the coded/raw video counterparts.
 * Uses DrainTrackingAudioSourceListener/QueueDrainingAudioSinkListener
 * directly (not createAudioSourceAndSink(), which hardcodes the no-op stub
 * listeners) so mGotFlushed/mGotDrained can actually be observed. No
 * "primer" push needed: audio sinks have no grey-IDR-style synthesis
 * quirk on their first-ever frame. ──────────────────────────────────── */

static void testCxxAudioSourceFlushDiscardsUnprocessedFrames()
{
	char inPath[512];
	PDRAW_GET_ASSET_PATH(inPath, ASSET_WAV_MONO, s_assets_sourcesink_audio);

	struct adef_format format = {};
	std::vector<struct mbuf_audio_frame *> inputFrames;
	std::vector<struct mbuf_mem *> inputMems;
	collectWavFrames(inPath, &format, &inputFrames, &inputMems);
	CU_ASSERT_FATAL(inputFrames.size() >= 3);

	TestPompLoop loop;
	AudioSourceMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	struct pdraw_audio_source_params sourceParams = {};
	sourceParams.playback_type = PDRAW_PLAYBACK_TYPE_REPLAY;
	sourceParams.audio.format = format;

	DrainTrackingAudioSourceListener sourceListener;
	IPdraw::IAudioSource *source = nullptr;
	int ret = session->createAudioSource(
		&sourceParams, &sourceListener, &source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source);
	auto sourceOwner = std::unique_ptr<IPdraw::IAudioSource>(source);
	mediaListener.mSource = source;

	bool gotMediaAdded = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotMediaAdded; });
	CU_ASSERT_TRUE_FATAL(gotMediaAdded);

	QueueDrainingAudioSinkListener sinkListener;
	IPdraw::IAudioSink *sink = nullptr;
	ret = session->createAudioSink(
		mediaListener.mMediaId, &sinkListener, &sink);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(sink);
	auto sinkOwner = std::unique_ptr<IPdraw::IAudioSink>(sink);

	struct mbuf_audio_frame_queue *inQueue = source->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(inQueue);
	struct mbuf_audio_frame_queue *outQueue = sink->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(outQueue);
	sinkListener.mQueue = outQueue;

	/* Push 3 frames back-to-back with NO pump() in between: they sit
	 * unprocessed in inQueue. */
	for (int i = 0; i < 3; i++) {
		ret = mbuf_audio_frame_queue_push(inQueue, inputFrames[i]);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
	}

	ret = source->flush();
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	bool gotFlushed = loop.pumpUntil(
		[&sourceListener]() { return sourceListener.mGotFlushed; },
		15000);
	CU_ASSERT_TRUE_FATAL(gotFlushed);

	/* Nothing made it to the sink: all 3 queued frames were discarded
	 * outright, never forwarded. */
	struct mbuf_audio_frame *leftover = nullptr;
	CU_ASSERT_NOT_EQUAL(mbuf_audio_frame_queue_pop(outQueue, &leftover), 0);

	for (auto *frame : inputFrames)
		mbuf_audio_frame_unref(frame);
	for (auto *mem : inputMems)
		mbuf_mem_unref(mem);

	sinkOwner.reset();
	sourceOwner.reset();
	stopSessionAndWait(&loop, session, &mediaListener);
}


static void testCxxAudioSourceDrainForwardsUnprocessedFrames()
{
	char inPath[512];
	PDRAW_GET_ASSET_PATH(inPath, ASSET_WAV_MONO, s_assets_sourcesink_audio);

	struct adef_format format = {};
	std::vector<struct mbuf_audio_frame *> inputFrames;
	std::vector<struct mbuf_mem *> inputMems;
	collectWavFrames(inPath, &format, &inputFrames, &inputMems);
	CU_ASSERT_FATAL(inputFrames.size() >= 3);

	TestPompLoop loop;
	AudioSourceMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	struct pdraw_audio_source_params sourceParams = {};
	sourceParams.playback_type = PDRAW_PLAYBACK_TYPE_REPLAY;
	sourceParams.audio.format = format;

	DrainTrackingAudioSourceListener sourceListener;
	IPdraw::IAudioSource *source = nullptr;
	int ret = session->createAudioSource(
		&sourceParams, &sourceListener, &source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source);
	auto sourceOwner = std::unique_ptr<IPdraw::IAudioSource>(source);
	mediaListener.mSource = source;

	bool gotMediaAdded = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotMediaAdded; });
	CU_ASSERT_TRUE_FATAL(gotMediaAdded);

	QueueDrainingAudioSinkListener sinkListener;
	IPdraw::IAudioSink *sink = nullptr;
	ret = session->createAudioSink(
		mediaListener.mMediaId, &sinkListener, &sink);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(sink);
	auto sinkOwner = std::unique_ptr<IPdraw::IAudioSink>(sink);

	struct mbuf_audio_frame_queue *inQueue = source->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(inQueue);
	struct mbuf_audio_frame_queue *outQueue = sink->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(outQueue);
	sinkListener.mQueue = outQueue;

	/* Same unpumped burst as above. */
	for (int i = 0; i < 3; i++) {
		ret = mbuf_audio_frame_queue_push(inQueue, inputFrames[i]);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
	}

	/* drain(discard=false) calls process() synchronously, right there in
	 * the call: by the time drain() returns, all 3 frames have already
	 * been forwarded all the way to the sink's queue. outputChannel->
	 * drain() is called AFTER process(), so by the time
	 * onAudioSinkDrain() fires on sinkListener, the 3 frames are already
	 * sitting in outQueue -- checked via sinkListener.mDrainedFrames
	 * (populated by that very callback), NOT by popping outQueue
	 * ourselves afterward, since the listener itself must pop the queue
	 * as part of draining it. */
	ret = source->drain();
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	bool gotDrained = loop.pumpUntil(
		[&sourceListener]() { return sourceListener.mGotDrained; },
		15000);
	CU_ASSERT_TRUE_FATAL(gotDrained);

	CU_ASSERT_EQUAL_FATAL(sinkListener.mDrainedFrames.size(), 3u);
	for (auto *frame : sinkListener.mDrainedFrames)
		mbuf_audio_frame_unref(frame);

	for (auto *frame : inputFrames)
		mbuf_audio_frame_unref(frame);
	for (auto *mem : inputMems)
		mbuf_mem_unref(mem);

	sinkOwner.reset();
	sourceOwner.reset();
	stopSessionAndWait(&loop, session, &mediaListener);
}


/* Exercises ExternalAudioSink::flushDone()'s mFlushDiscard mismatch path
 * (pdraw_external_audio_sink.cpp:282-287): the application acknowledges a
 * real flush (mFlushDiscard==true, set when the source called flush()) by
 * calling queueDrained() instead of queueFlushed() -- see
 * MismatchAckAudioSinkListener::onAudioSinkFlush() above. The mismatch only
 * logs a warning; flushDone()'s actual channel operation still follows
 * mFlushDiscard, not the discard argument the application passed, so
 * channel->flushDone() runs (matching the flush the source actually
 * requested, since only Channel::mFlushPending -- not mDrainPending -- was
 * ever set) and the source still receives onAudioSourceFlushed(), never
 * onAudioSourceDrained(). Uses the same LIVE-mode + canned
 * adef_pcm_16b_44100hz_stereo construction as
 * testCxxAudioSourceSinkSwitchMediaId below.
 *
 * Two preconditions, both confirmed missing by real runs (on the raw/coded
 * video counterparts of this test) before it could ever reach
 * onAudioSinkFlush():
 * 1. The SOURCE's own frame queue must be non-empty when flush() is called:
 *    per ExternalAudioSource::flush(), an empty queue takes an early
 *    "already flushed, nothing to do" shortcut that completes its own flush
 *    via an idle callback without ever calling outputChannel->flush() --
 *    the sink never even receives the downstream FLUSH event.
 * 2. The SINK's own FlushingState must already be UNFLUSHED (not its default
 *    FLUSHED) when the channel's FLUSH event arrives: ExternalAudioSink::
 *    flush() has the exact same "already flushed, nothing to do" shortcut,
 *    and it stays in FLUSHED (the default) until it has actually received a
 *    frame at least once (the only place that flips it to UNFLUSHED,
 *    pdraw_external_audio_sink.cpp:552) -- a source-side discard, which
 *    never forwards anything, does not do this.
 *
 * So: prime the sink first with a real frame that is pumped all the way
 * through, THEN push a second frame unpumped into the source's own queue
 * before calling flush(). */
static void testCxxAudioSinkFlushAckMismatchFollowsRequestedDiscardState()
{
	TestPompLoop loop;
	AudioSourceMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	struct pdraw_audio_source_params sourceParams = {};
	sourceParams.playback_type = PDRAW_PLAYBACK_TYPE_LIVE;
	sourceParams.audio.format = adef_pcm_16b_44100hz_stereo;

	DrainTrackingAudioSourceListener sourceListener;
	IPdraw::IAudioSource *source = nullptr;
	int ret = session->createAudioSource(
		&sourceParams, &sourceListener, &source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source);
	auto sourceOwner = std::unique_ptr<IPdraw::IAudioSource>(source);
	mediaListener.mSource = source;

	bool gotMediaAdded = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotMediaAdded; });
	CU_ASSERT_TRUE_FATAL(gotMediaAdded);

	MismatchAckAudioSinkListener sinkListener;
	IPdraw::IAudioSink *sink = nullptr;
	ret = session->createAudioSink(
		mediaListener.mMediaId, &sinkListener, &sink);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(sink);
	auto sinkOwner = std::unique_ptr<IPdraw::IAudioSink>(sink);

	bool gotSinkMediaAdded = loop.pumpUntil(
		[&sinkListener]() { return sinkListener.mGotMediaAdded; });
	CU_ASSERT_TRUE_FATAL(gotSinkMediaAdded);

	struct mbuf_audio_frame_queue *inQueue = source->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(inQueue);
	struct mbuf_audio_frame_queue *outQueue = sink->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(outQueue);

	/* Prime: push a frame and let it fully arrive at the sink -- this alone
	 * flips the sink's FlushingState to UNFLUSHED (precondition 2 above),
	 * and that state persists regardless of the pop below (nothing resets
	 * it back to FLUSHED before flush() is called). push() does not
	 * transfer ownership (see mbuf_audio_frame.h), so each frame below is
	 * unreffed right after its own push -- the queue holds its own
	 * reference regardless. */
	struct mbuf_audio_frame *primerFrame =
		makeDummyAudioFrame(adef_pcm_16b_44100hz_stereo, UINT64_C(1));
	ret = mbuf_audio_frame_queue_push(inQueue, primerFrame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	mbuf_audio_frame_unref(primerFrame);
	struct mbuf_audio_frame *poppedPrimer = nullptr;
	bool gotPrimer = loop.pumpUntil(
		[&]() {
			return mbuf_audio_frame_queue_pop(outQueue,
							  &poppedPrimer) == 0;
		},
		5000);
	CU_ASSERT_TRUE_FATAL(gotPrimer);
	mbuf_audio_frame_unref(poppedPrimer);

	/* Unpumped push: sits in the source's own queue so flush() below finds
	 * it non-empty (precondition 1 above). */
	struct mbuf_audio_frame *frame =
		makeDummyAudioFrame(adef_pcm_16b_44100hz_stereo, UINT64_C(2));
	ret = mbuf_audio_frame_queue_push(inQueue, frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	mbuf_audio_frame_unref(frame);

	/* Real flush (discard=true): sets mFlushDiscard=true on the sink. */
	ret = source->flush();
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	/* onAudioSinkFlush() fires and acks with queueDrained() instead of
	 * queueFlushed() -- yet the source still sees its flush complete, not
	 * a drain. */
	bool gotFlushed = loop.pumpUntil(
		[&sourceListener]() { return sourceListener.mGotFlushed; },
		15000);
	CU_ASSERT_TRUE_FATAL(gotFlushed);
	CU_ASSERT_FALSE(sourceListener.mGotDrained);

	sinkOwner.reset();
	sourceOwner.reset();
	stopSessionAndWait(&loop, session, &mediaListener);
}


/* Verifies setMediaId() / getMediaId() on IAudioSink by creating two audio
 * sources, attaching the sink to source1, then switching it to source2
 * mid-stream -- the IAudioSink counterpart of
 * testCxxVideoRendererSwitchMediaId (test_pipeline_renderer_video.cpp),
 * testCxxCodedSourceSinkSwitchMediaId (test_pipeline_sourcesink_coded.cpp)
 * and testCxxRawSourceSinkSwitchMediaId (test_pipeline_sourcesink_raw.cpp).
 *
 * Why this test exists: every other sink in this file is created either
 * with media_id=0 (auto-attach to whatever media turns up next) or with an
 * already-existing media's id at creation time -- both resolved through the
 * same broadcast+filter path (Session::onElementStateChanged /
 * PipelineFactory::onOutputMediaAdded -> addAllMediaToAudioSink() /
 * addMediaToAllToAudioSinks(), filtered internally by
 * ExternalAudioSink::addInputMedia()'s mTargetMediaId check). Neither ever
 * calls Session::addMediaToAudioSink(unsigned int mediaId, Sink*) /
 * PipelineFactory's same-named overload -- the single-lookup-by-id path,
 * 0%-covered per the coverage report. The only caller of that path is
 * ExternalAudioSink::idleRenewMedia(), itself only scheduled by
 * setMediaId(): setMediaId(m2) sets mTargetMediaId=m2 and schedules
 * idleRenewMedia() on the pomp loop; when that idle fires:
 * removeInputMedia(source1's media) -> mSession->addMediaToAudioSink(m2,
 * this) -> addInputMedia(source2's media). Both onAudioSinkMediaRemoved and
 * onAudioSinkMediaAdded fire synchronously within that same idle callback.
 *
 * Uses the same LIVE-mode + canned adef_pcm_16b_44100hz_stereo construction
 * as test_pipeline_renderer_audio.cpp (no WAV/AAC asset file needed), since
 * this test only cares about the id-switch plumbing, not real frame data. */
static void testCxxAudioSourceSinkSwitchMediaId()
{
	TestPompLoop loop;
	AudioSourceMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	struct pdraw_audio_source_params sourceParams = {};
	sourceParams.playback_type = PDRAW_PLAYBACK_TYPE_LIVE;
	sourceParams.audio.format = adef_pcm_16b_44100hz_stereo;

	/* --- Source 1 and its media --- */
	IPdraw::IAudioSource *source1 = nullptr;
	int ret = session->createAudioSource(
		&sourceParams, &g_stub_audio_source_listener, &source1);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source1);
	auto source1Owner = std::unique_ptr<IPdraw::IAudioSource>(source1);
	mediaListener.mSource = source1;

	bool gotMedia1 = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotMediaAdded; });
	CU_ASSERT_TRUE_FATAL(gotMedia1);
	unsigned int media1Id = mediaListener.mMediaId;
	CU_ASSERT_NOT_EQUAL_FATAL(media1Id, 0u);

	/* --- Source 2 and its media --- */
	mediaListener.mGotMediaAdded = false;
	IPdraw::IAudioSource *source2 = nullptr;
	ret = session->createAudioSource(
		&sourceParams, &g_stub_audio_source_listener, &source2);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source2);
	auto source2Owner = std::unique_ptr<IPdraw::IAudioSource>(source2);
	mediaListener.mSource = source2;

	bool gotMedia2 = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotMediaAdded; });
	CU_ASSERT_TRUE_FATAL(gotMedia2);
	unsigned int media2Id = mediaListener.mMediaId;
	CU_ASSERT_NOT_EQUAL_FATAL(media2Id, 0u);
	CU_ASSERT_NOT_EQUAL_FATAL(media1Id, media2Id);

	/* --- Sink on source1's media --- */
	AudioSinkTrackingListener sinkListener;
	IPdraw::IAudioSink *sink = nullptr;
	ret = session->createAudioSink(media1Id, &sinkListener, &sink);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(sink);
	auto sinkOwner = std::unique_ptr<IPdraw::IAudioSink>(sink);

	bool gotSinkMedia1 = loop.pumpUntil(
		[&sinkListener]() { return sinkListener.mGotMediaAdded; });
	CU_ASSERT_TRUE_FATAL(gotSinkMedia1);
	CU_ASSERT_EQUAL(sink->getMediaId(), media1Id);

	/* --- Switch the sink to source2 --- */
	sinkListener.mGotMediaAdded = false;
	sinkListener.mGotMediaRemoved = false;

	ret = sink->setMediaId(media2Id);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	bool gotRemoved = loop.pumpUntil(
		[&sinkListener]() { return sinkListener.mGotMediaRemoved; });
	CU_ASSERT_TRUE_FATAL(gotRemoved);
	/* setMediaId() does not force a resync, so restart=false. */
	CU_ASSERT_FALSE(sinkListener.mLastRemovedRestart);

	bool gotAdded2 = loop.pumpUntil(
		[&sinkListener]() { return sinkListener.mGotMediaAdded; });
	CU_ASSERT_TRUE_FATAL(gotAdded2);
	CU_ASSERT_EQUAL(sink->getMediaId(), media2Id);

	sinkOwner.reset();
	source1Owner.reset();
	source2Owner.reset();
	stopSessionAndWait(&loop, session, &mediaListener);
}


/* Verifies ExternalAudioSink::setMediaId()'s early-return branch
 * (pdraw_external_audio_sink.cpp:216-217): calling setMediaId() with the
 * SAME id as the sink's current mTargetMediaId returns 0 immediately without
 * scheduling idleRenewMedia() -- unlike testCxxAudioSourceSinkSwitchMediaId
 * above (different id), which does trigger a media remove+re-add. Same
 * LIVE-mode construction as that test. */
static void testCxxAudioSinkSetMediaIdSameIdIsNoop()
{
	TestPompLoop loop;
	AudioSourceMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	struct pdraw_audio_source_params sourceParams = {};
	sourceParams.playback_type = PDRAW_PLAYBACK_TYPE_LIVE;
	sourceParams.audio.format = adef_pcm_16b_44100hz_stereo;

	IPdraw::IAudioSource *source = nullptr;
	int ret = session->createAudioSource(
		&sourceParams, &g_stub_audio_source_listener, &source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source);
	auto sourceOwner = std::unique_ptr<IPdraw::IAudioSource>(source);
	mediaListener.mSource = source;

	bool gotMediaAdded = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotMediaAdded; });
	CU_ASSERT_TRUE_FATAL(gotMediaAdded);

	AudioSinkTrackingListener sinkListener;
	IPdraw::IAudioSink *sink = nullptr;
	ret = session->createAudioSink(
		mediaListener.mMediaId, &sinkListener, &sink);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(sink);
	auto sinkOwner = std::unique_ptr<IPdraw::IAudioSink>(sink);

	bool gotSinkMediaAdded = loop.pumpUntil(
		[&sinkListener]() { return sinkListener.mGotMediaAdded; });
	CU_ASSERT_TRUE_FATAL(gotSinkMediaAdded);

	unsigned int mediaId = sink->getMediaId();
	CU_ASSERT_NOT_EQUAL_FATAL(mediaId, 0u);

	sinkListener.mGotMediaAdded = false;
	sinkListener.mGotMediaRemoved = false;

	ret = sink->setMediaId(mediaId);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	/* No idleRenewMedia() is scheduled: pump for a short, bounded time and
	 * confirm neither callback fires (contrast with
	 * testCxxAudioSourceSinkSwitchMediaId, where switching to a different
	 * id always fires both). */
	bool gotEvent = loop.pumpUntil(
		[&sinkListener]() {
			return sinkListener.mGotMediaRemoved ||
			       sinkListener.mGotMediaAdded;
		},
		300);
	CU_ASSERT_FALSE(gotEvent);
	CU_ASSERT_EQUAL(sink->getMediaId(), mediaId);

	sinkOwner.reset();
	sourceOwner.reset();
	stopSessionAndWait(&loop, session, &mediaListener);
}


/* Regression test for two real use-after-free bugs found via ASan on the
 * coded video counterpart of this scenario
 * (testCxxCodedSourceSinkAbruptSessionDestructionDoesNotCrash,
 * test_pipeline_sourcesink_coded.cpp -- see that test's comment for the
 * full history). Destroying the whole Session abruptly (skipping stop())
 * while a source and sink are both still fully wired up used to crash
 * twice: (1) ExternalAudioSource::~ExternalAudioSource() freeing its
 * mOutputMedia while a channel was still attached, walked as a dangling
 * pointer by ~Source()'s own removeOutputPorts() right after; (2) once
 * fixed, the still-attached Sink crashing in turn on its own mInputMedia,
 * pointing at that same already-freed Media object. Both are now fixed by
 * Source::teardownOutputChannels() (pdraw_source.cpp), called from the
 * destructor before removeOutputPort(): it synchronously runs the
 * channel->teardown() round-trip (Sink::onChannelTeardown() ->
 * removeInputMedia()) while the media is still valid, before the source
 * frees it -- which as a side effect also means the sink's own "input media
 * has not been removed" branch is no longer reachable even here (its
 * mInputMedia is already null by the time its own destructor runs). This
 * test therefore targets Source::teardownOutputChannels(), not that
 * branch. */
static void testCxxAudioSourceSinkAbruptSessionDestructionDoesNotCrash()
{
	AudioSourceMediaListener mediaListener;
	AudioSinkTrackingListener sinkListener;
	std::unique_ptr<IPdraw::IAudioSource> sourceOwner;
	std::unique_ptr<IPdraw::IAudioSink> sinkOwner;

	{
		TestPompLoop loop;
		TestSession testSession(&loop, &mediaListener);
		IPdraw *session = testSession.get();

		struct pdraw_audio_source_params sourceParams = {};
		sourceParams.playback_type = PDRAW_PLAYBACK_TYPE_LIVE;
		sourceParams.audio.format = adef_pcm_16b_44100hz_stereo;

		IPdraw::IAudioSource *source = nullptr;
		int ret = session->createAudioSource(
			&sourceParams, &g_stub_audio_source_listener, &source);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
		CU_ASSERT_PTR_NOT_NULL_FATAL(source);
		sourceOwner = std::unique_ptr<IPdraw::IAudioSource>(source);
		mediaListener.mSource = source;

		bool gotMediaAdded = loop.pumpUntil([&mediaListener]() {
			return mediaListener.mGotMediaAdded;
		});
		CU_ASSERT_TRUE_FATAL(gotMediaAdded);

		IPdraw::IAudioSink *sink = nullptr;
		ret = session->createAudioSink(
			mediaListener.mMediaId, &sinkListener, &sink);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
		CU_ASSERT_PTR_NOT_NULL_FATAL(sink);
		sinkOwner = std::unique_ptr<IPdraw::IAudioSink>(sink);

		bool gotSinkMediaAdded = loop.pumpUntil([&sinkListener]() {
			return sinkListener.mGotMediaAdded;
		});
		CU_ASSERT_TRUE_FATAL(gotSinkMediaAdded);

		/* Deliberately no stop()/stopSessionAndWait() call here:
		 * testSession and loop are destroyed right below, at the end
		 * of this block, forcing ~Session() to run while mState is
		 * still READY -- the only way to reach the scenario described
		 * above. */
	}

	/* If we get here, the abrupt teardown did not crash. The sink was
	 * synchronously notified of its media's removal as part of the
	 * source's own destructor -- confirming the fix actually ran, not
	 * just that nothing crashed. */
	CU_ASSERT_TRUE(sinkListener.mGotMediaRemoved);
}


/* Covers ExternalAudioSink::onChannelReconfigure() (0% per the coverage
 * report). No production audio element ever originates a RECONFIGURE
 * downstream event: it is only ever sent by StreamDemuxer::VideoMedia
 * (RTSP codec-info/RTCP-event/goodbye) and VipcSource (raw video format
 * change) -- both video-only -- or re-broadcast by FilterElement::
 * onChannelReconfigure() to its own output media when ITS OWN input channel
 * receives one (AudioEncoder/AudioDecoder are FilterElements, but nothing
 * upstream of them ever sends the event either). So there is no real audio
 * pipeline that can reach this callback naturally; it must be injected
 * directly on the sink's real input Channel via Channel::
 * sendDownstreamEvent(), a public method meant for exactly this (see
 * test_channel_audio.cpp, which does the same on a mock sink). Reaching the
 * real production ExternalAudioSink object (not a mock) requires casting
 * through AudioSinkWrapper, the same test-only-cast pattern already used
 * for AlsaSourceWrapper::getAlsaSource() in test_pipeline_alsa_source.cpp.
 *
 * ExternalAudioSink::onChannelReconfigure() sets a private mPendingRestart
 * flag with no other observable side effect; the only place it surfaces is
 * as the `restart` argument of the NEXT onAudioSinkMediaRemoved() call (see
 * ExternalAudioSink::removeInputMedia()). testCxxAudioSourceSinkSwitchMediaId
 * above already confirms restart=false on the ordinary setMediaId() path
 * (no reconfigure in between); this test forces the same removal but with a
 * RECONFIGURE injected first, and asserts restart=true instead. */
static void testCxxAudioSinkReconfigureSetsPendingRestartOnRemoval()
{
	TestPompLoop loop;
	AudioSourceMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	struct pdraw_audio_source_params sourceParams = {};
	sourceParams.playback_type = PDRAW_PLAYBACK_TYPE_LIVE;
	sourceParams.audio.format = adef_pcm_16b_44100hz_stereo;

	/* --- Source 1: the media the sink will be reconfigured on --- */
	IPdraw::IAudioSource *source1 = nullptr;
	int ret = session->createAudioSource(
		&sourceParams, &g_stub_audio_source_listener, &source1);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source1);
	auto source1Owner = std::unique_ptr<IPdraw::IAudioSource>(source1);
	mediaListener.mSource = source1;

	bool gotMedia1 = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotMediaAdded; });
	CU_ASSERT_TRUE_FATAL(gotMedia1);
	unsigned int media1Id = mediaListener.mMediaId;
	CU_ASSERT_NOT_EQUAL_FATAL(media1Id, 0u);

	/* --- Source 2: only used as setMediaId()'s target, to force the
	 * sink to remove media1 (and thus report mPendingRestart). --- */
	mediaListener.mGotMediaAdded = false;
	IPdraw::IAudioSource *source2 = nullptr;
	ret = session->createAudioSource(
		&sourceParams, &g_stub_audio_source_listener, &source2);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source2);
	auto source2Owner = std::unique_ptr<IPdraw::IAudioSource>(source2);
	mediaListener.mSource = source2;

	bool gotMedia2 = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotMediaAdded; });
	CU_ASSERT_TRUE_FATAL(gotMedia2);
	unsigned int media2Id = mediaListener.mMediaId;
	CU_ASSERT_NOT_EQUAL_FATAL(media1Id, media2Id);

	/* --- Sink on source1's media --- */
	AudioSinkTrackingListener sinkListener;
	IPdraw::IAudioSink *sink = nullptr;
	ret = session->createAudioSink(media1Id, &sinkListener, &sink);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(sink);
	auto sinkOwner = std::unique_ptr<IPdraw::IAudioSink>(sink);

	bool gotSinkMedia = loop.pumpUntil(
		[&sinkListener]() { return sinkListener.mGotMediaAdded; });
	CU_ASSERT_TRUE_FATAL(gotSinkMedia);

	/* --- Inject RECONFIGURE directly on the sink's real input channel
	 * --- */
	auto *sinkWrapper = static_cast<AudioSinkWrapper *>(sink);
	Sink *rawSink = sinkWrapper->getSink();
	CU_ASSERT_PTR_NOT_NULL_FATAL(rawSink);
	Media *media1 = rawSink->getInputMedia(0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(media1);
	Channel *channel = rawSink->getInputChannel(media1);
	CU_ASSERT_PTR_NOT_NULL_FATAL(channel);
	int sendRet = channel->sendDownstreamEvent(
		Channel::DownstreamEvent::RECONFIGURE);
	CU_ASSERT_EQUAL(sendRet, 0);

	/* --- Force removal of media1 by switching the sink to media2 --- */
	sinkListener.mGotMediaRemoved = false;
	ret = sink->setMediaId(media2Id);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	bool gotRemoved = loop.pumpUntil(
		[&sinkListener]() { return sinkListener.mGotMediaRemoved; });
	CU_ASSERT_TRUE_FATAL(gotRemoved);
	CU_ASSERT_TRUE(sinkListener.mLastRemovedRestart);

	sinkOwner.reset();
	source1Owner.reset();
	source2Owner.reset();
	stopSessionAndWait(&loop, session, &mediaListener);
}


/* ── C-wrapper listener callback tests (C API) ──
 * Self-contained pdraw_new() fixtures with counting C callbacks.
 * SessionCbState / make_pdraw() are shared via test_pipeline_common.hpp. */

struct AudioSrcCbState {
	int flushedCount = 0;
	int drainedCount = 0;
};

static void audio_src_flushed_cb(struct pdraw * /*p*/,
				 struct pdraw_audio_source * /*s*/,
				 void *ud)
{
	static_cast<AudioSrcCbState *>(ud)->flushedCount++;
}

static void audio_src_drained_cb(struct pdraw * /*p*/,
				 struct pdraw_audio_source * /*s*/,
				 void *ud)
{
	static_cast<AudioSrcCbState *>(ud)->drainedCount++;
}

struct AudioSinkCbState {
	struct pdraw *p = nullptr;
	int mediaAddedCount = 0;
	int flushCount = 0;
	int drainCount = 0;
	int mediaRemovedCount = 0;
};

static void audio_sink_media_added_cb(struct pdraw * /*p*/,
				      struct pdraw_audio_sink * /*sk*/,
				      const struct pdraw_media_info * /*info*/,
				      void *ud)
{
	static_cast<AudioSinkCbState *>(ud)->mediaAddedCount++;
}

static void
audio_sink_media_removed_cb(struct pdraw * /*p*/,
			    struct pdraw_audio_sink * /*sk*/,
			    const struct pdraw_media_info * /*info*/,
			    int /*restart*/,
			    void *ud)
{
	static_cast<AudioSinkCbState *>(ud)->mediaRemovedCount++;
}

static void
audio_sink_flush_cb(struct pdraw *p, struct pdraw_audio_sink *sk, void *ud)
{
	static_cast<AudioSinkCbState *>(ud)->flushCount++;
	struct mbuf_audio_frame_queue *q = pdraw_audio_sink_get_queue(p, sk);
	if (q != nullptr)
		mbuf_audio_frame_queue_flush(q);
	pdraw_audio_sink_queue_flushed(p, sk);
}

static void
audio_sink_drain_cb(struct pdraw *p, struct pdraw_audio_sink *sk, void *ud)
{
	static_cast<AudioSinkCbState *>(ud)->drainCount++;
	/* Pop and unref any leftover frames, then acknowledge drain */
	struct mbuf_audio_frame_queue *q = pdraw_audio_sink_get_queue(p, sk);
	if (q != nullptr) {
		struct mbuf_audio_frame *frame = nullptr;
		while (mbuf_audio_frame_queue_pop(q, &frame) == 0)
			mbuf_audio_frame_unref(frame);
	}
	pdraw_audio_sink_queue_drained(p, sk);
}


static void testCAudioSourceListenerFlushedDrained()
{
	TestPompLoop loop;
	SessionCbState sessState;
	struct pdraw *p = nullptr;
	CU_ASSERT_TRUE_FATAL(make_pdraw(loop, sessState, &p));

	struct pdraw_audio_source_cbs cbs = {};
	cbs.flushed = audio_src_flushed_cb;
	cbs.drained = audio_src_drained_cb;
	AudioSrcCbState srcState;

	struct pdraw_audio_source_params params = {};
	struct pdraw_audio_source *src = nullptr;
	int ret = pdraw_audio_source_new(p, &params, &cbs, &srcState, &src);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(src);

	ret = pdraw_audio_source_flush(p, src);
	CU_ASSERT_EQUAL(ret, 0);
	bool gotFlushed = loop.pumpUntil(
		[&srcState]() { return srcState.flushedCount >= 1; });
	CU_ASSERT_TRUE(gotFlushed);
	CU_ASSERT_EQUAL(srcState.flushedCount, 1);

	ret = pdraw_audio_source_drain(p, src);
	CU_ASSERT_EQUAL(ret, 0);
	bool gotDrained = loop.pumpUntil(
		[&srcState]() { return srcState.drainedCount >= 1; });
	CU_ASSERT_TRUE(gotDrained);
	CU_ASSERT_EQUAL(srcState.drainedCount, 1);

	pdraw_audio_source_destroy(p, src);
	pdraw_destroy(p);
}


static void testCAudioSinkListenerCallbacks()
{
	TestPompLoop loop;
	SessionCbState sessState;
	struct pdraw *p = nullptr;
	CU_ASSERT_TRUE_FATAL(make_pdraw(loop, sessState, &p));

	/* Create audio source with counting callbacks.
	 * Use a concrete PCM format so that the dummy frames we push later
	 * pass adef_format_intersect() in the source's input filter. */
	struct pdraw_audio_source_cbs srcCbs = {};
	srcCbs.flushed = audio_src_flushed_cb;
	srcCbs.drained = audio_src_drained_cb;
	AudioSrcCbState srcState;
	struct pdraw_audio_source_params srcParams = {};
	srcParams.audio.format = adef_pcm_16b_44100hz_mono;
	struct pdraw_audio_source *src = nullptr;
	int ret =
		pdraw_audio_source_new(p, &srcParams, &srcCbs, &srcState, &src);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(src);

	/* Wait for session media_added to get the audio media ID */
	bool gotAdded = loop.pumpUntil(
		[&sessState]() { return sessState.lastMediaId != 0; });
	CU_ASSERT_TRUE_FATAL(gotAdded);
	unsigned int mediaId = sessState.lastMediaId;

	/* Create audio sink on that media */
	AudioSinkCbState sinkState;
	sinkState.p = p;
	struct pdraw_audio_sink_cbs sinkCbs = {};
	sinkCbs.media_added = audio_sink_media_added_cb;
	sinkCbs.media_removed = audio_sink_media_removed_cb;
	sinkCbs.flush = audio_sink_flush_cb;
	sinkCbs.drain = audio_sink_drain_cb;
	struct pdraw_audio_sink *snk = nullptr;
	ret = pdraw_audio_sink_new(p, mediaId, &sinkCbs, &sinkState, &snk);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(snk);

	/* Wait for the sink's own onAudioSinkMediaAdded */
	bool gotSinkAdded = loop.pumpUntil(
		[&sinkState]() { return sinkState.mediaAddedCount >= 1; });
	CU_ASSERT_TRUE(gotSinkAdded);

	/* Flush: push frame, pump once to deliver to sink, then flush */
	{
		struct mbuf_audio_frame_queue *inQ =
			pdraw_audio_source_get_queue(p, src);
		CU_ASSERT_PTR_NOT_NULL_FATAL(inQ);
		struct mbuf_audio_frame *f = PdrawTest::makeDummyAudioFrame(
			44100 /* 1 s at 44100 Hz */);
		ret = mbuf_audio_frame_queue_push(inQ, f);
		CU_ASSERT_EQUAL(ret, 0);
		mbuf_audio_frame_unref(f);
	}
	loop.runOnce(); /* queueEventCb → processFrame → frame in sink queue */
	ret = pdraw_audio_source_flush(p, src);
	CU_ASSERT_EQUAL(ret, 0);
	bool gotFlush = loop.pumpUntil([&sinkState, &srcState]() {
		return sinkState.flushCount >= 1 && srcState.flushedCount >= 1;
	});
	CU_ASSERT_TRUE(gotFlush);
	CU_ASSERT_EQUAL(sinkState.flushCount, 1);
	CU_ASSERT_EQUAL(srcState.flushedCount, 1);

	/* Drain: process() inside flush(false) delivers the frame first */
	{
		struct mbuf_audio_frame_queue *inQ =
			pdraw_audio_source_get_queue(p, src);
		CU_ASSERT_PTR_NOT_NULL_FATAL(inQ);
		struct mbuf_audio_frame *f = PdrawTest::makeDummyAudioFrame(
			88200 /* 2 s at 44100 Hz */);
		ret = mbuf_audio_frame_queue_push(inQ, f);
		CU_ASSERT_EQUAL(ret, 0);
		mbuf_audio_frame_unref(f);
	}
	ret = pdraw_audio_source_drain(p, src);
	CU_ASSERT_EQUAL(ret, 0);
	bool gotDrain = loop.pumpUntil([&sinkState, &srcState]() {
		return sinkState.drainCount >= 1 && srcState.drainedCount >= 1;
	});
	CU_ASSERT_TRUE(gotDrain);
	CU_ASSERT_EQUAL(sinkState.drainCount, 1);
	CU_ASSERT_EQUAL(srcState.drainedCount, 1);

	/* Destroy source: fires onAudioSinkMediaRemoved */
	pdraw_audio_source_destroy(p, src);
	src = nullptr;
	bool gotRemoved = loop.pumpUntil(
		[&sinkState]() { return sinkState.mediaRemovedCount >= 1; });
	CU_ASSERT_TRUE(gotRemoved);

	pdraw_audio_sink_destroy(p, snk);
	pdraw_destroy(p);
}


/* Session-wide listener capturing the full pdraw_audio_info exposed by
 * onMediaAdded(), not just the media id (unlike AudioSourceMediaListener
 * above) -- needed to inspect audio.aac_lc.asc/asclen below. */
class AacAscMediaListener : public IPdraw::Listener {
public:
	void stopResponse(IPdraw * /*p*/, int status) override
	{
		mStopStatus = status;
		mGotStopResponse = true;
	}

	void onMediaAdded(IPdraw * /*p*/,
			  const struct pdraw_media_info *info,
			  void *elementUserData) override
	{
		if (elementUserData != mSource)
			return;
		mAudioInfo = info->audio;
		mGotMediaAdded = true;
	}

	void onMediaRemoved(IPdraw * /*p*/,
			    const struct pdraw_media_info * /*i*/,
			    void * /*u*/) override
	{
	}

	void onSocketCreated(IPdraw * /*p*/, int /*fd*/) override {}

	void *mSource = nullptr;
	bool mGotMediaAdded = false;
	struct pdraw_audio_info mAudioInfo = {};
	bool mGotStopResponse = false;
	int mStopStatus = 0;
};


/* Regression test for ExternalAudioSource::start() (pdraw_external_audio_
 * source.cpp) now copying the AAC-LC ASC from pdraw_audio_source_params into
 * the output AudioMedia via AudioMedia::setAacAsc(), mirroring what
 * ExternalCodedVideoSource::start() already did for H.264/H.265 SPS/PPS via
 * setPs() (pdraw_external_coded_video_source.cpp:178-186). Before this fix,
 * AudioMedia::mAacAsc stayed empty for any synthetic IAudioSource regardless
 * of what the caller passed in audio.aac_lc, so AudioMedia::fillMediaInfo()
 * always exposed asclen == 0 downstream -- in particular,
 * IsobmffMuxerAudioMedia::setup() (pdraw_muxer_record_isobmff_media.cpp)
 * would then always fail: mp4_mux_track_set_audio_specific_config() rejects
 * asc_size == 0 (libmp4/src/mp4_mux.c:1490), making it impossible to record
 * a synthetic AAC-LC IAudioSource with an ISOBMFF muxer. */
static void testCxxAudioSourceExposesAacLcAsc()
{
	static const uint8_t kAsc[] = {0x12, 0x08}; /* AAC-LC 44100Hz mono */

	TestPompLoop loop;
	AacAscMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	struct pdraw_audio_source_params sourceParams = {};
	sourceParams.playback_type = PDRAW_PLAYBACK_TYPE_REPLAY;
	sourceParams.audio.format = adef_aac_lc_16b_44100hz_mono_raw;
	memcpy(sourceParams.audio.aac_lc.asc, kAsc, sizeof(kAsc));
	sourceParams.audio.aac_lc.asclen = sizeof(kAsc);

	IPdraw::IAudioSource *source = nullptr;
	int ret = session->createAudioSource(
		&sourceParams, &g_stub_audio_source_listener, &source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source);
	auto sourceOwner = std::unique_ptr<IPdraw::IAudioSource>(source);
	mediaListener.mSource = source;

	bool gotMediaAdded = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotMediaAdded; });
	CU_ASSERT_TRUE_FATAL(gotMediaAdded);

	CU_ASSERT_EQUAL(mediaListener.mAudioInfo.aac_lc.asclen, sizeof(kAsc));
	CU_ASSERT_EQUAL(
		memcmp(mediaListener.mAudioInfo.aac_lc.asc, kAsc, sizeof(kAsc)),
		0);

	sourceOwner.reset();
	stopSessionAndWait(&loop, session, &mediaListener);
}


static void testCxxAudioSourceWrapperGuardsAfterElementCleared()
{
	TestPompLoop loop;
	AudioSourceMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	struct pdraw_audio_source_params sourceParams = {};
	sourceParams.playback_type = PDRAW_PLAYBACK_TYPE_LIVE;
	sourceParams.audio.format = adef_pcm_16b_44100hz_stereo;

	IPdraw::IAudioSource *source = nullptr;
	int ret = session->createAudioSource(
		&sourceParams, &g_stub_audio_source_listener, &source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source);
	auto sourceOwner = std::unique_ptr<IPdraw::IAudioSource>(source);
	mediaListener.mSource = source;

	bool gotMediaAdded = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotMediaAdded; });
	CU_ASSERT_TRUE_FATAL(gotMediaAdded);

	stopSessionAndWait(&loop, session, &mediaListener);

	CU_ASSERT_PTR_NULL(source->getQueue());
	CU_ASSERT_EQUAL(source->flush(), -EPROTO);
	CU_ASSERT_EQUAL(source->drain(), -EPROTO);
}


static void testCxxAudioSinkWrapperGuardsAfterElementCleared()
{
	TestPompLoop loop;
	AudioSourceMediaListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	struct pdraw_audio_source_params sourceParams = {};
	sourceParams.playback_type = PDRAW_PLAYBACK_TYPE_LIVE;
	sourceParams.audio.format = adef_pcm_16b_44100hz_stereo;

	IPdraw::IAudioSource *source = nullptr;
	int ret = session->createAudioSource(
		&sourceParams, &g_stub_audio_source_listener, &source);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source);
	mediaListener.mSource = source;

	bool gotMediaAdded = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotMediaAdded; });
	CU_ASSERT_TRUE_FATAL(gotMediaAdded);

	IPdraw::IAudioSink *sink = nullptr;
	ret = session->createAudioSink(
		mediaListener.mMediaId, &g_stub_audio_sink_listener, &sink);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(sink);
	auto sinkOwner = std::unique_ptr<IPdraw::IAudioSink>(sink);

	auto sourceOwner = std::unique_ptr<IPdraw::IAudioSource>(source);

	stopSessionAndWait(&loop, session, &mediaListener);

	CU_ASSERT_EQUAL(sink->setMediaId(0), -EPROTO);
	CU_ASSERT_EQUAL(sink->getMediaId(), (unsigned int)-EPROTO);
	CU_ASSERT_PTR_NULL(sink->getQueue());
	CU_ASSERT_EQUAL(sink->queueFlushed(), -EPROTO);
	CU_ASSERT_EQUAL(sink->queueDrained(), -EPROTO);
}


} /* anonymous namespace */


CU_TestInfo g_pdraw_test_pipeline_sourcesink_audio[] = {
	{FN("testCxxAudioSourceSinkRoundtripWavMono"),
	 testCxxAudioSourceSinkRoundtripWavMono},
	{FN("testCxxAudioSourceSinkRoundtripWavStereo"),
	 testCxxAudioSourceSinkRoundtripWavStereo},
	{FN("testCxxAudioSourceSinkRoundtripAacMono"),
	 testCxxAudioSourceSinkRoundtripAacMono},
	{FN("testCxxAudioSourceSinkRoundtripAacStereo"),
	 testCxxAudioSourceSinkRoundtripAacStereo},
	{FN("testCxxAudioSourceRejectsFormatMismatch"),
	 testCxxAudioSourceRejectsFormatMismatch},
	{FN("testCxxAudioSourceRejectsNonMonotonicTimestamp"),
	 testCxxAudioSourceRejectsNonMonotonicTimestamp},
	{FN("testCxxAudioSourceFlushDiscardsUnprocessedFrames"),
	 testCxxAudioSourceFlushDiscardsUnprocessedFrames},
	{FN("testCxxAudioSourceDrainForwardsUnprocessedFrames"),
	 testCxxAudioSourceDrainForwardsUnprocessedFrames},
	{FN("testCxxAudioSinkFlushAckMismatchFollowsRequestedDiscardState"),
	 testCxxAudioSinkFlushAckMismatchFollowsRequestedDiscardState},
	{FN("testCxxAudioSourceSinkSwitchMediaId"),
	 testCxxAudioSourceSinkSwitchMediaId},
	{FN("testCxxAudioSinkSetMediaIdSameIdIsNoop"),
	 testCxxAudioSinkSetMediaIdSameIdIsNoop},
	{FN("testCxxAudioSourceSinkAbruptSessionDestructionDoesNotCrash"),
	 testCxxAudioSourceSinkAbruptSessionDestructionDoesNotCrash},
	{FN("testCxxAudioSinkReconfigureSetsPendingRestartOnRemoval"),
	 testCxxAudioSinkReconfigureSetsPendingRestartOnRemoval},
	{FN("testCAudioSourceListenerFlushedDrained"),
	 testCAudioSourceListenerFlushedDrained},
	{FN("testCAudioSinkListenerCallbacks"),
	 testCAudioSinkListenerCallbacks},
	{FN("testCxxAudioSourceExposesAacLcAsc"),
	 testCxxAudioSourceExposesAacLcAsc},
	{FN("testCxxAudioSourceWrapperGuardsAfterElementCleared"),
	 testCxxAudioSourceWrapperGuardsAfterElementCleared},
	{FN("testCxxAudioSinkWrapperGuardsAfterElementCleared"),
	 testCxxAudioSinkWrapperGuardsAfterElementCleared},
	CU_TEST_INFO_NULL,
};
