/**
 * Parrot Drones Audio and Video Vector library
 * libpdraw unit tests — shared stubs for API input-validation tests
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

/* NOTE: Include this header only AFTER defining ULOG_TAG and including
 * test_fixtures.hpp (which itself requires ULOG_TAG). */

#pragma once

#include "pdraw/pdraw.h"
#include "pdraw/pdraw.hpp"

#include <CUnit/CUnit.h>
#include <audio-defs/adefs.h>
#include <media-buffers/mbuf_audio_frame.h>
#include <media-buffers/mbuf_mem_generic.h>
#include <media-buffers/mbuf_raw_video_frame.h>
#include <video-defs/vdefs.h>

#include <sys/stat.h>
#include <sys/uio.h>
#include <unistd.h>

#include <futils/futils.h>

#include <cerrno>
#include <climits>
#include <cstdio>
#include <cstdlib>

/* Assert the expected errno for a null-parameter rejection on feature-gated
 * APIs.  When the feature is compiled in, the null guard fires and returns
 * -EINVAL.  When the feature is absent, the function short-circuits and
 * returns -ENOSYS before any null check runs. */
#ifdef BUILD_LIBVIDEO_IPC
#	define CU_ASSERT_VIPC_NULL_GUARD(ret) CU_ASSERT_EQUAL((ret), -EINVAL)
#else
#	define CU_ASSERT_VIPC_NULL_GUARD(ret) CU_ASSERT_EQUAL((ret), -ENOSYS)
#endif

#ifdef PDRAW_USE_ALSA
#	define CU_ASSERT_ALSA_NULL_GUARD(ret) CU_ASSERT_EQUAL((ret), -EINVAL)
#else
#	define CU_ASSERT_ALSA_NULL_GUARD(ret) CU_ASSERT_EQUAL((ret), -ENOSYS)
#endif

/* VideoDecoder::addInputMedia() intersects the media's coded format against
 * vdec_get_supported_input_formats(), which only advertises JPEG/JFIF when
 * libvideo-decode was built with its TurboJPEG implementation (see
 * CONFIG_VDEC_TURBOJPEG, propagated to this test build as -DVDEC_TURBOJPEG
 * in libpdraw/atom.mk). Without it, pdraw_sink rejects the format and
 * addInputMedia() returns -ENOSYS before start() is ever reached. */
#ifdef VDEC_TURBOJPEG
#	define CU_ASSERT_JPEG_DECODER_INPUT_GUARD(ret)                        \
		CU_ASSERT_EQUAL((ret), 0)
#else
#	define CU_ASSERT_JPEG_DECODER_INPUT_GUARD(ret)                        \
		CU_ASSERT_EQUAL((ret), -ENOSYS)
#endif

/* Audio renderer creation (with otherwise fully valid parameters) only
 * succeeds when ALSA support is compiled in: AudioRenderer::create() returns
 * nullptr without PDRAW_USE_ALSA, which Session::createAudioRenderer()
 * surfaces as -EPROTO. AlsaAudioRenderer's constructor does not open the
 * actual ALSA device (deferred to start()), so this holds regardless of
 * whether a real ALSA device is present on the host.
 * There is no equivalent CU_ASSERT_VIDEO_RENDERER_CREATE_GUARD: unlike
 * AlsaAudioRenderer, GlVideoRenderer's constructor issues a real GL call
 * (see the comment in test_api_renderer_video.cpp), so success also depends
 * on a bound GL context being available — not just the compile flag. */
#ifdef PDRAW_USE_ALSA
#	define CU_ASSERT_AUDIO_RENDERER_CREATE_GUARD(ret)                     \
		CU_ASSERT_EQUAL((ret), 0)
#else
#	define CU_ASSERT_AUDIO_RENDERER_CREATE_GUARD(ret)                     \
		CU_ASSERT_EQUAL((ret), -EPROTO)
#endif

/* Not guaranteed to come from <climits> on every target (POSIX only
 * requires it when a filesystem has a fixed max path length) — same
 * defensive fallback as libmp4/src/mp4_recovery.c. */
#ifndef PATH_MAX
#	define PATH_MAX 4096
#endif

/* NAS mount used by other Parrot test suites (see libmp4/tests/mp4_test.h)
 * for real media assets too large to embed in the repository. Overridable
 * via the ASSETS_ROOT environment variable, e.g. on a machine where the NAS
 * is mounted elsewhere or unavailable and a local copy is used instead. */
#define PDRAW_TEST_ASSETS_ROOT "/mnt/DFS/MULTIMEDIA_DATA"

/* Local cache of NAS assets fetched via PDRAW_GET_ASSET_PATH. Once an asset
 * has been copied here (this run or a previous one), tests read the local
 * copy instead of the NAS mount, so a transient NAS/NFS glitch during
 * decode/mux can no longer turn into a flaky failure mid-test — only the
 * initial fetch still touches the NAS, and that fetch is retried. */
#define PDRAW_TEST_ASSETS_CACHE_ROOT "/tmp/pdraw_test_assets_cache"

/* Creates all missing parent directories of filePath (`mkdir -p
 * $(dirname filePath)`). Returns 0 on success, -errno otherwise. */
inline int pdrawTestMkdirsForFile(const char *filePath)
{
	char path[PATH_MAX];
	snprintf(path, sizeof(path), "%s", filePath);

	for (char *p = path + 1; *p != '\0'; p++) {
		if (*p != '/')
			continue;
		*p = '\0';
		if (mkdir(path, 0755) != 0 && errno != EEXIST)
			return -errno;
		*p = '/';
	}
	return 0;
}

/* Copies srcPath to dstPath, creating dstPath's parent directories as
 * needed. Writes to a temporary file first and renames it into place on
 * success, so a failed or concurrently-running copy never leaves a partial
 * file at dstPath. Returns 0 on success, -errno otherwise. */
inline int pdrawTestCopyFile(const char *srcPath, const char *dstPath)
{
	int ret = pdrawTestMkdirsForFile(dstPath);
	if (ret != 0)
		return ret;

	FILE *src = fopen(srcPath, "rb");
	if (src == NULL)
		return -errno;

	char tmpPath[PATH_MAX];
	snprintf(tmpPath,
		 sizeof(tmpPath),
		 "%s.tmp.%d",
		 dstPath,
		 static_cast<int>(getpid()));

	FILE *dst = fopen(tmpPath, "wb");
	if (dst == NULL) {
		int err = -errno;
		fclose(src);
		return err;
	}

	char buf[1 << 20];
	size_t n;
	int err = 0;
	while ((n = fread(buf, 1, sizeof(buf), src)) > 0) {
		if (fwrite(buf, 1, n, dst) != n) {
			err = -EIO;
			break;
		}
	}
	if (err == 0 && ferror(src))
		err = -EIO;

	fclose(src);
	if (fclose(dst) != 0 && err == 0)
		err = -errno;

	if (err != 0) {
		unlink(tmpPath);
		return err;
	}

	if (rename(tmpPath, dstPath) != 0) {
		err = -errno;
		unlink(tmpPath);
		return err;
	}

	return 0;
}

/* Fetches nasPath into the local cache at cachePath. If cachePath is
 * already readable (fetched by an earlier test or run), it is reused as-is
 * with no NAS access at all. Otherwise retries the NAS access + copy a few
 * times with a short backoff, to absorb momentary NAS/NFS glitches instead
 * of failing the test outright. Returns 0 on success, -errno otherwise
 * (the last error observed, once every attempt has failed). */
inline int pdrawTestFetchAsset(const char *nasPath, const char *cachePath)
{
	if (access(cachePath, R_OK) == 0)
		return 0;

	/* 1 immediate attempt + backoff before each retry. */
	static const useconds_t backoffsUs[] = {100000, 300000, 800000};
	const size_t maxAttempts = FUTILS_SIZEOF_ARRAY(backoffsUs) + 1;
	int lastErr = -ENOENT;

	for (size_t attempt = 0; attempt < maxAttempts; attempt++) {
		if (attempt > 0)
			usleep(backoffsUs[attempt - 1]);

		if (access(nasPath, R_OK) != 0) {
			lastErr = -errno;
			continue;
		}

		lastErr = pdrawTestCopyFile(nasPath, cachePath);
		if (lastErr == 0)
			return 0;
	}

	return lastErr;
}

/* Resolve the _index-th relative_path entry of _array (a struct array with
 * a `relative_path` member) into a NUL-terminated absolute path in _dest.
 *
 * _dest always points into the local cache (PDRAW_TEST_ASSETS_CACHE_ROOT),
 * fetched on first use via pdrawTestFetchAsset() (with retries — see there)
 * from either ASSETS_ROOT if set, or the NAS mount otherwise -- ASSETS_ROOT
 * is not assumed to be a fast/local path: it may be a network mount too
 * (e.g. a GVFS/FUSE SMB mount, observed to be much slower than a kernel CIFS
 * mount or the local cache for the scattered small reads MP4 demuxing does),
 * so every source is routed through the same cache-once-read-many-times
 * path. Fails the current test (CU_ASSERT_FATAL, i.e. skips the rest of it
 * rather than crashing the whole binary) if the asset could not be made
 * available — typically because neither ASSETS_ROOT nor the NAS is
 * reachable. */
#define PDRAW_GET_ASSET_PATH(_dest, _index, _array)                            \
	do {                                                                   \
		CU_ASSERT_FATAL((_index) < FUTILS_SIZEOF_ARRAY(_array));       \
		const char *_pdraw_assets_root = getenv("ASSETS_ROOT");        \
		char _pdraw_src_path[PATH_MAX];                                \
		snprintf(_pdraw_src_path,                                      \
			 sizeof(_pdraw_src_path),                              \
			 "%s/%s",                                              \
			 (_pdraw_assets_root != NULL)                          \
				 ? _pdraw_assets_root                          \
				 : PDRAW_TEST_ASSETS_ROOT,                     \
			 (_array)[(_index)].relative_path);                    \
		snprintf((_dest),                                              \
			 sizeof(_dest),                                        \
			 "%s/%s",                                              \
			 PDRAW_TEST_ASSETS_CACHE_ROOT,                         \
			 (_array)[(_index)].relative_path);                    \
		int _pdraw_asset_ret =                                         \
			pdrawTestFetchAsset(_pdraw_src_path, (_dest));         \
		CU_ASSERT_EQUAL_FATAL(_pdraw_asset_ret, 0);                    \
	} while (0)

namespace PdrawTest {

using namespace Pdraw;


/* ── C stubs — all callback fields set to NULL ──────────────────────────── */
/* Suitable for testing the cbs==nullptr guard and smoke tests.
 * Do NOT use for sources/sinks that require mandatory callbacks (use the
 * g_valid_*_cbs variants below instead). */

extern const struct pdraw_cbs g_stub_pdraw_cbs;
extern const struct pdraw_demuxer_cbs g_stub_demuxer_cbs;
extern const struct pdraw_muxer_cbs g_stub_muxer_cbs;
extern const struct pdraw_video_renderer_cbs g_stub_video_renderer_cbs;
extern const struct pdraw_audio_renderer_cbs g_stub_audio_renderer_cbs;
extern const struct pdraw_vipc_source_cbs g_stub_vipc_source_cbs;
extern const struct pdraw_coded_video_source_cbs g_stub_coded_video_source_cbs;
extern const struct pdraw_raw_video_source_cbs g_stub_raw_video_source_cbs;
extern const struct pdraw_coded_video_sink_cbs g_stub_coded_video_sink_cbs;
extern const struct pdraw_raw_video_sink_cbs g_stub_raw_video_sink_cbs;
extern const struct pdraw_alsa_source_cbs g_stub_alsa_source_cbs;
extern const struct pdraw_audio_source_cbs g_stub_audio_source_cbs;
extern const struct pdraw_audio_sink_cbs g_stub_audio_sink_cbs;
extern const struct pdraw_video_encoder_cbs g_stub_video_encoder_cbs;
extern const struct pdraw_video_scaler_cbs g_stub_video_scaler_cbs;
extern const struct pdraw_audio_encoder_cbs g_stub_audio_encoder_cbs;

/* ── C valid stubs — mandatory callbacks populated ──────────────────────── */
/* For sources/sinks that mandate cbs->flushed or cbs->flush to be non-null. */

extern const struct pdraw_coded_video_source_cbs g_valid_coded_video_source_cbs;
extern const struct pdraw_raw_video_source_cbs g_valid_raw_video_source_cbs;
extern const struct pdraw_coded_video_sink_cbs g_valid_coded_video_sink_cbs;
extern const struct pdraw_raw_video_sink_cbs g_valid_raw_video_sink_cbs;
extern const struct pdraw_audio_source_cbs g_valid_audio_source_cbs;
extern const struct pdraw_audio_sink_cbs g_valid_audio_sink_cbs;


/* ── C++ Listener stub classes ───────────────────────────────────────────── */
/* All pure-virtual methods implemented as no-ops (or minimum return values).
 * Non-copyable per PDRAW_DISABLE_COPY but can be used by address. */

class StubPdrawListener : public IPdraw::Listener {
public:
	void stopResponse(IPdraw * /*p*/, int /*s*/) override {}
	void onMediaAdded(IPdraw * /*p*/,
			  const struct pdraw_media_info * /*i*/,
			  void * /*u*/) override
	{
	}
	void onMediaRemoved(IPdraw * /*p*/,
			    const struct pdraw_media_info * /*i*/,
			    void * /*u*/) override
	{
	}
	void onSocketCreated(IPdraw * /*p*/, int /*fd*/) override {}
};

class StubDemuxerListener : public IPdraw::IDemuxer::Listener {
public:
	void demuxerOpenResponse(IPdraw * /*p*/,
				 IPdraw::IDemuxer * /*d*/,
				 int /*s*/) override
	{
	}
	void demuxerCloseResponse(IPdraw * /*p*/,
				  IPdraw::IDemuxer * /*d*/,
				  int /*s*/) override
	{
	}
	void onDemuxerUnrecoverableError(IPdraw * /*p*/,
					 IPdraw::IDemuxer * /*d*/) override
	{
	}
	int demuxerSelectMedia(IPdraw * /*p*/,
			       IPdraw::IDemuxer * /*d*/,
			       const struct pdraw_demuxer_media * /*m*/,
			       size_t /*c*/,
			       uint32_t /*sel*/) override
	{
		return -ENOSYS;
	}
	void demuxerReadyToPlay(IPdraw * /*p*/,
				IPdraw::IDemuxer * /*d*/,
				bool /*r*/) override
	{
	}
	void onDemuxerEndOfRange(IPdraw * /*p*/,
				 IPdraw::IDemuxer * /*d*/,
				 uint64_t /*ts*/) override
	{
	}
	void demuxerPlayResponse(IPdraw * /*p*/,
				 IPdraw::IDemuxer * /*d*/,
				 int /*s*/,
				 uint64_t /*ts*/,
				 float /*sp*/) override
	{
	}
	void demuxerPauseResponse(IPdraw * /*p*/,
				  IPdraw::IDemuxer * /*d*/,
				  int /*s*/,
				  uint64_t /*ts*/) override
	{
	}
	void demuxerSeekResponse(IPdraw * /*p*/,
				 IPdraw::IDemuxer * /*d*/,
				 int /*s*/,
				 uint64_t /*ts*/,
				 float /*sp*/) override
	{
	}
};

class StubMuxerListener : public IPdraw::IMuxer::Listener {
public:
	void onMuxerConnectionStateChanged(
		IPdraw * /*p*/,
		IPdraw::IMuxer * /*m*/,
		enum pdraw_muxer_connection_state /*cs*/,
		enum pdraw_muxer_disconnection_reason /*dr*/) override
	{
	}
	void onMuxerMediaReady(IPdraw * /*p*/,
			       IPdraw::IMuxer * /*m*/,
			       const char * /*path*/,
			       const struct iovec * /*iov*/,
			       int /*cnt*/) override
	{
	}
	void onMuxerMediaSaved(IPdraw * /*p*/,
			       IPdraw::IMuxer * /*m*/,
			       const char * /*path*/) override
	{
	}
	void onMuxerUnrecoverableError(IPdraw * /*p*/,
				       IPdraw::IMuxer * /*m*/,
				       int /*s*/) override
	{
	}
	void muxerCloseResponse(IPdraw * /*p*/,
				IPdraw::IMuxer * /*m*/,
				int /*s*/) override
	{
	}
};

class StubVideoRendererListener : public IPdraw::IVideoRenderer::Listener {
public:
	void onVideoRendererMediaAdded(
		IPdraw * /*p*/,
		IPdraw::IVideoRenderer * /*r*/,
		const struct pdraw_media_info * /*i*/) override
	{
	}
	void onVideoRendererMediaRemoved(IPdraw * /*p*/,
					 IPdraw::IVideoRenderer * /*r*/,
					 const struct pdraw_media_info * /*i*/,
					 bool /*restart*/) override
	{
	}
	void onVideoRenderReady(IPdraw * /*p*/,
				IPdraw::IVideoRenderer * /*r*/) override
	{
	}
	int loadVideoTexture(IPdraw * /*p*/,
			     IPdraw::IVideoRenderer * /*r*/,
			     unsigned int /*tw*/,
			     unsigned int /*th*/,
			     const struct pdraw_media_info * /*mi*/,
			     struct mbuf_raw_video_frame * /*frame*/,
			     const void * /*ud*/,
			     size_t /*udlen*/) override
	{
		return -ENOSYS;
	}
	int renderVideoOverlay(
		IPdraw * /*p*/,
		IPdraw::IVideoRenderer * /*r*/,
		const struct pdraw_rect * /*rp*/,
		const struct pdraw_rect * /*cp*/,
		const float * /*vm*/,
		const float * /*pm*/,
		const struct pdraw_media_info * /*mi*/,
		struct vmeta_frame * /*fm*/,
		const struct pdraw_video_frame_extra * /*fe*/) override
	{
		return -ENOSYS;
	}
};

class StubAudioRendererListener : public IPdraw::IAudioRenderer::Listener {
public:
	void onAudioRendererMediaAdded(
		IPdraw * /*p*/,
		IPdraw::IAudioRenderer * /*r*/,
		const struct pdraw_media_info * /*i*/) override
	{
	}
	void onAudioRendererMediaRemoved(
		IPdraw * /*p*/,
		IPdraw::IAudioRenderer * /*r*/,
		const struct pdraw_media_info * /*i*/) override
	{
	}
};

class StubVipcSourceListener : public IPdraw::IVipcSource::Listener {
public:
	void
	vipcSourceReadyToPlay(IPdraw * /*p*/,
			      IPdraw::IVipcSource * /*s*/,
			      bool /*r*/,
			      enum pdraw_vipc_source_eos_reason /*e*/) override
	{
	}
	void vipcSourcePlayResponse(IPdraw * /*p*/,
				    IPdraw::IVipcSource * /*s*/) override
	{
	}
	void vipcSourcePauseResponse(IPdraw * /*p*/,
				     IPdraw::IVipcSource * /*s*/) override
	{
	}
	bool
	vipcSourceFramerateChanged(IPdraw * /*p*/,
				   IPdraw::IVipcSource * /*s*/,
				   const struct vdef_frac * /*prev*/,
				   const struct vdef_frac * /*next*/) override
	{
		return false;
	}
	void vipcSourceConfigured(IPdraw * /*p*/,
				  IPdraw::IVipcSource * /*s*/,
				  int /*st*/,
				  const struct vdef_format_info * /*info*/,
				  const struct vdef_rectf * /*crop*/) override
	{
	}
	void
	vipcSourceFrameReady(IPdraw * /*p*/,
			     IPdraw::IVipcSource * /*s*/,
			     struct mbuf_raw_video_frame * /*frame*/) override
	{
	}
	bool
	vipcSourceEndOfStream(IPdraw * /*p*/,
			      IPdraw::IVipcSource * /*s*/,
			      enum pdraw_vipc_source_eos_reason /*e*/) override
	{
		return false;
	}
};

class StubCodedVideoSourceListener
		: public IPdraw::ICodedVideoSource::Listener {
public:
	void
	onCodedVideoSourceFlushed(IPdraw * /*p*/,
				  IPdraw::ICodedVideoSource * /*s*/) override
	{
	}
	void
	onCodedVideoSourceDrained(IPdraw * /*p*/,
				  IPdraw::ICodedVideoSource * /*s*/) override
	{
	}
};

class StubRawVideoSourceListener : public IPdraw::IRawVideoSource::Listener {
public:
	void onRawVideoSourceFlushed(IPdraw * /*p*/,
				     IPdraw::IRawVideoSource * /*s*/) override
	{
	}
	void onRawVideoSourceDrained(IPdraw * /*p*/,
				     IPdraw::IRawVideoSource * /*s*/) override
	{
	}
};

class StubCodedVideoSinkListener : public IPdraw::ICodedVideoSink::Listener {
public:
	void onCodedVideoSinkMediaAdded(
		IPdraw * /*p*/,
		IPdraw::ICodedVideoSink * /*sk*/,
		const struct pdraw_media_info * /*i*/) override
	{
	}
	void onCodedVideoSinkMediaRemoved(IPdraw * /*p*/,
					  IPdraw::ICodedVideoSink * /*sk*/,
					  const struct pdraw_media_info * /*i*/,
					  bool /*restart*/) override
	{
	}
	void onCodedVideoSinkFlush(IPdraw * /*p*/,
				   IPdraw::ICodedVideoSink * /*sk*/) override
	{
	}
	void onCodedVideoSinkDrain(IPdraw * /*p*/,
				   IPdraw::ICodedVideoSink * /*sk*/) override
	{
	}
	void onCodedVideoSinkSessionMetaUpdate(
		IPdraw * /*p*/,
		IPdraw::ICodedVideoSink * /*sk*/,
		const struct vmeta_session * /*meta*/) override
	{
	}
};

class StubRawVideoSinkListener : public IPdraw::IRawVideoSink::Listener {
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
				 IPdraw::IRawVideoSink * /*sk*/) override
	{
	}
	void onRawVideoSinkDrain(IPdraw * /*p*/,
				 IPdraw::IRawVideoSink * /*sk*/) override
	{
	}
	void onRawVideoSinkSessionMetaUpdate(
		IPdraw * /*p*/,
		IPdraw::IRawVideoSink * /*sk*/,
		const struct vmeta_session * /*meta*/) override
	{
	}
};

class StubAlsaSourceListener : public IPdraw::IAlsaSource::Listener {
public:
	void
	alsaSourceReadyToPlay(IPdraw * /*p*/,
			      IPdraw::IAlsaSource * /*s*/,
			      bool /*r*/,
			      enum pdraw_alsa_source_eos_reason /*e*/) override
	{
	}
	void alsaSourcePlayResponse(IPdraw * /*p*/,
				    IPdraw::IAlsaSource * /*s*/) override
	{
	}
	void alsaSourcePauseResponse(IPdraw * /*p*/,
				     IPdraw::IAlsaSource * /*s*/) override
	{
	}
	void alsaSourceFrameReady(IPdraw * /*p*/,
				  IPdraw::IAlsaSource * /*s*/,
				  struct mbuf_audio_frame * /*f*/) override
	{
	}
};

class StubAudioSourceListener : public IPdraw::IAudioSource::Listener {
public:
	void onAudioSourceFlushed(IPdraw * /*p*/,
				  IPdraw::IAudioSource * /*s*/) override
	{
	}
	void onAudioSourceDrained(IPdraw * /*p*/,
				  IPdraw::IAudioSource * /*s*/) override
	{
	}
};

class StubAudioSinkListener : public IPdraw::IAudioSink::Listener {
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
	void onAudioSinkFlush(IPdraw * /*p*/,
			      IPdraw::IAudioSink * /*sk*/) override
	{
	}
	void onAudioSinkDrain(IPdraw * /*p*/,
			      IPdraw::IAudioSink * /*sk*/) override
	{
	}
};

class StubVideoEncoderListener : public IPdraw::IVideoEncoder::Listener {
public:
	void
	videoEncoderFrameOutput(IPdraw * /*p*/,
				IPdraw::IVideoEncoder * /*e*/,
				struct mbuf_coded_video_frame * /*f*/) override
	{
	}
	void videoEncoderFramePreRelease(
		IPdraw * /*p*/,
		IPdraw::IVideoEncoder * /*e*/,
		struct mbuf_coded_video_frame * /*f*/) override
	{
	}
};

class StubAudioEncoderListener : public IPdraw::IAudioEncoder::Listener {
public:
	void audioEncoderFrameOutput(IPdraw * /*p*/,
				     IPdraw::IAudioEncoder * /*e*/,
				     struct mbuf_audio_frame * /*f*/) override
	{
	}
	void
	audioEncoderFramePreRelease(IPdraw * /*p*/,
				    IPdraw::IAudioEncoder * /*e*/,
				    struct mbuf_audio_frame * /*f*/) override
	{
	}
};

class StubVideoScalerListener : public IPdraw::IVideoScaler::Listener {
public:
	void
	videoScalerFrameOutput(IPdraw * /*p*/,
			       IPdraw::IVideoScaler * /*sc*/,
			       struct mbuf_raw_video_frame * /*f*/) override
	{
	}
};


/* ── Shared stub listener instances ────────────────────────────────────── */

extern StubPdrawListener g_stub_pdraw_listener;
extern StubDemuxerListener g_stub_demuxer_listener;
extern StubMuxerListener g_stub_muxer_listener;
extern StubVideoRendererListener g_stub_video_renderer_listener;
extern StubAudioRendererListener g_stub_audio_renderer_listener;
extern StubVipcSourceListener g_stub_vipc_source_listener;
extern StubCodedVideoSourceListener g_stub_coded_video_source_listener;
extern StubRawVideoSourceListener g_stub_raw_video_source_listener;
extern StubCodedVideoSinkListener g_stub_coded_video_sink_listener;
extern StubRawVideoSinkListener g_stub_raw_video_sink_listener;
extern StubAlsaSourceListener g_stub_alsa_source_listener;
extern StubAudioSourceListener g_stub_audio_source_listener;
extern StubAudioSinkListener g_stub_audio_sink_listener;
extern StubVideoEncoderListener g_stub_video_encoder_listener;
extern StubAudioEncoderListener g_stub_audio_encoder_listener;
extern StubVideoScalerListener g_stub_video_scaler_listener;


/* ── Shared dummy frame factories ────────────────────────────────────────
 * Used by Tier-C C-wrapper listener callback tests to put a source into
 * UNFLUSHED state so that flush()/drain() calls actually propagate
 * downstream through connected channels to the sink.
 * (When FlushingState == FLUSHED and the input queue is empty, the source
 * takes an early-return fast path and never calls channel->flush().)
 *
 * Declared inline so including the header in multiple translation units
 * does not cause ODR violations. */

inline struct mbuf_raw_video_frame *makeDummyRawVideoFrame(uint64_t timestamp)
{
	struct vdef_raw_frame fi = {};
	fi.format = vdef_raw8;
	fi.info.timescale = 1000000;
	fi.info.timestamp = timestamp;
	fi.info.resolution.width = 1;
	fi.info.resolution.height = 1;
	fi.info.bit_depth = 8;

	struct mbuf_raw_video_frame *frame = nullptr;
	int ret = mbuf_raw_video_frame_new(&fi, &frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(frame);

	unsigned int nPlanes = vdef_get_raw_frame_plane_count(&vdef_raw8);
	if (nPlanes == 0)
		nPlanes = 1;
	const size_t planeSize = 64;
	struct mbuf_mem *mem = nullptr;
	ret = mbuf_mem_generic_new(planeSize * nPlanes, &mem);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	for (unsigned int i = 0; i < nPlanes; i++) {
		ret = mbuf_raw_video_frame_set_plane(
			frame, i, mem, i * planeSize, planeSize);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
	}
	mbuf_mem_unref(mem);

	ret = mbuf_raw_video_frame_finalize(frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	return frame;
}


inline struct mbuf_audio_frame *makeDummyAudioFrame(uint64_t timestamp)
{
	struct adef_frame fi = {};
	fi.format = adef_pcm_16b_44100hz_mono;
	fi.info.timescale = fi.format.sample_rate;
	fi.info.timestamp = timestamp;

	struct mbuf_audio_frame *frame = nullptr;
	int ret = mbuf_audio_frame_new(&fi, &frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(frame);

	const size_t bufLen = 16;
	struct mbuf_mem *mem = nullptr;
	ret = mbuf_mem_generic_new(bufLen, &mem);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	ret = mbuf_audio_frame_set_buffer(frame, mem, 0, bufLen);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	mbuf_mem_unref(mem);

	ret = mbuf_audio_frame_finalize(frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	return frame;
}


} /* namespace PdrawTest */


/* ── Fallback handle re-declarations ───────────────────────────────────── */
/* pdraw.h forward-declares these structs as incomplete types.  The actual
 * definitions are file-static in pdraw_wrapper.cpp.  We provide matching
 * definitions here so that tests for GL-dependent elements (video renderer)
 * can allocate a handle directly without requiring a GL context.
 * Defined at global scope to match the type expected by the C API.
 * Layout must stay in sync with pdraw_wrapper.cpp. */

struct pdraw_video_renderer {
	Pdraw::IPdraw::IVideoRenderer *impl;
};

struct pdraw_audio_renderer {
	Pdraw::IPdraw::IAudioRenderer *impl;
};
