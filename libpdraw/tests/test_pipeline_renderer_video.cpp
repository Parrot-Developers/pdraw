/**
 * Parrot Drones Audio and Video Vector library
 * libpdraw unit tests — GL video renderer against a real, self-contained
 * offscreen EGL context (Tier B, self-contained fixture)
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

/* Requires PDRAW_USE_GL, and (for now) a Linux desktop target: the EGL
 * offscreen-context helper below only covers that case (see TestGlContext).
 * On any other configuration this file compiles to an empty suite.
 *
 * test_api_renderer_video.cpp already covers createVideoRenderer()/
 * render()/etc. null-arg guards, plus the not-compiled-in path
 * (-EPROTO without PDRAW_USE_GL). Its testCxxCreateValid() explicitly
 * documents why it stops short of exercising a real render(): libpdraw's
 * GlVideoRenderer::setup() (called synchronously from the constructor)
 * issues a real glGetIntegerv(GL_FRAMEBUFFER_BINDING, ...), which requires
 * a GL context already current on the calling thread -- something a bare
 * CUnit run does not have. This file supplies that context itself (a 1x1
 * offscreen EGL pbuffer, GLES3), closing that gap with a genuine end-to-end
 * render of a real frame.
 *
 * Confirmed by reading pdraw_renderer_video_gl.cpp/pdraw_gl_video.cpp before
 * writing this: neither file ever calls any EGL/GLX/window-system function
 * itself -- libpdraw always assumes the application already made a context
 * current, both at createVideoRenderer() time and at every render() call.
 * The only in-tree example of doing so for pdraw rendering is
 * libpdraw-streamsharing/src/pdraw_streamsharing_gl_context_egl.c; the
 * TestGlContext class below mirrors it (simplified: one context, one
 * thread, no refcounting, no separate bind()/unbind() -- construction binds,
 * destruction unbinds). */

#if defined(PDRAW_USE_GL) && defined(__linux__) && !defined(__ANDROID__)
#	define PDRAW_TEST_RENDERER_VIDEO_GL_ENABLED 1
#endif

#define ULOG_TAG pdraw_test_pipeline_renderer_video
#include "test_api_common.hpp"
#include "test_common.h"
#include "test_fixtures.hpp"
#include "test_pipeline_common.hpp"

ULOG_DECLARE_TAG(ULOG_TAG);

using namespace Pdraw;
using namespace PdrawTest;

#ifdef PDRAW_TEST_RENDERER_VIDEO_GL_ENABLED

#	include "pdraw_renderer_video.hpp"
#	include "pdraw_renderer_video_gl.hpp"

#	include <media-buffers/mbuf_mem_generic.h>
#	include <media-buffers/mbuf_raw_video_frame.h>
#	include <video-defs/vdefs.h>
#	include <video-metadata/vmeta_frame_proto.h>
#	include <video-streaming/vstrm.h>

#	include <EGL/egl.h>
#	include <GLES2/gl2.h>

#	include <cstdlib>
#	include <cstring>
#	include <string>
#	include <vector>


/* Anonymous namespace: internal linkage for the listener/helper classes
 * below, to avoid ODR violations with identically-named classes defined
 * differently in sibling test_*.cpp files linked into the same binary (see
 * test_pipeline_vipc_source.cpp for the bug this pattern was found to fix). */
namespace {

/* Session-wide listener recording every media added, so the raw video
 * source's own output media id can be found -- same pattern as every other
 * test_pipeline_*.cpp file (duplicated here rather than shared, per this
 * suite's convention). */
class MediaTrackingListener : public IPdraw::Listener {
public:
	struct Added {
		unsigned int id;
		enum vdef_frame_type videoFormat;
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


/* Tracks IVideoRenderer's own lifecycle events. loadVideoTexture()/
 * renderVideoOverlay() return -ENOSYS (before checking any argument, as
 * mandated by their doc comments) so that libpdraw's own internal texture
 * loading and rendering path is exercised instead of an application
 * override -- the whole point of this test. */
class VideoRendererTrackingListener : public IPdraw::IVideoRenderer::Listener {
public:
	void onVideoRendererMediaAdded(
		IPdraw * /*p*/,
		IPdraw::IVideoRenderer * /*r*/,
		const struct pdraw_media_info * /*info*/) override
	{
		mGotMediaAdded = true;
	}

	void
	onVideoRendererMediaRemoved(IPdraw * /*p*/,
				    IPdraw::IVideoRenderer * /*r*/,
				    const struct pdraw_media_info * /*info*/,
				    bool restart) override
	{
		mGotMediaRemoved = true;
		mLastRemovedRestart = restart;
	}

	void onVideoRenderReady(IPdraw * /*p*/,
				IPdraw::IVideoRenderer * /*r*/) override
	{
		mRenderReadyCount++;
	}

	int loadVideoTexture(IPdraw * /*p*/,
			     IPdraw::IVideoRenderer * /*r*/,
			     unsigned int /*textureWidth*/,
			     unsigned int /*textureHeight*/,
			     const struct pdraw_media_info * /*mediaInfo*/,
			     struct mbuf_raw_video_frame * /*frame*/,
			     const void * /*frameUserdata*/,
			     size_t /*frameUserdataLen*/) override
	{
		return -ENOSYS;
	}

	int renderVideoOverlay(
		IPdraw * /*p*/,
		IPdraw::IVideoRenderer * /*r*/,
		const struct pdraw_rect * /*renderPos*/,
		const struct pdraw_rect * /*contentPos*/,
		const float * /*viewMat*/,
		const float * /*projMat*/,
		const struct pdraw_media_info * /*mediaInfo*/,
		struct vmeta_frame * /*frameMeta*/,
		const struct pdraw_video_frame_extra * /*frameExtra*/) override
	{
		return -ENOSYS;
	}

	bool mGotMediaAdded = false;
	std::atomic<int> mRenderReadyCount{0};
	bool mGotMediaRemoved = false;
	bool mLastRemovedRestart = false;
};


/* Listener that enables external video texture loading path by returning 0 from
 * loadVideoTexture. */
class ExtLoadVideoTextureListener : public VideoRendererTrackingListener {
public:
	int loadVideoTexture(IPdraw * /*p*/,
			     IPdraw::IVideoRenderer * /*r*/,
			     unsigned int /*textureWidth*/,
			     unsigned int /*textureHeight*/,
			     const struct pdraw_media_info * /*mediaInfo*/,
			     struct mbuf_raw_video_frame * /*frame*/,
			     const void * /*frameUserdata*/,
			     size_t /*frameUserdataLen*/) override
	{
		return 0;
	}
};


/* Records drain/flush completion callbacks from a raw video source. */
class TrackingRawVideoSourceListener
		: public IPdraw::IRawVideoSource::Listener {
public:
	void onRawVideoSourceFlushed(IPdraw * /*p*/,
				     IPdraw::IRawVideoSource * /*s*/) override
	{
		mGotFlushed = true;
	}

	void onRawVideoSourceDrained(IPdraw * /*p*/,
				     IPdraw::IRawVideoSource * /*s*/) override
	{
		mGotDrained = true;
	}

	bool mGotFlushed = false;
	bool mGotDrained = false;
};


/* Listener that activates the external texture loading path by returning 0
 * (not -ENOSYS) from loadVideoTexture, and records the texture dimensions
 * the renderer computed.  The probe call made during construction passes
 * textureWidth=0; real calls pass the computed size, so we filter on width>0.
 * The listener does NOT write pixels to the texture: the FBO stays black, but
 * that is fine -- the test only verifies that the path was entered with the
 * correct dimensions.  renderVideoOverlay still returns -ENOSYS so the overlay
 * path stays disabled. */
class ExtTexCapturingRendererListener : public VideoRendererTrackingListener {
public:
	int loadVideoTexture(IPdraw * /*p*/,
			     IPdraw::IVideoRenderer * /*r*/,
			     unsigned int textureWidth,
			     unsigned int textureHeight,
			     const struct pdraw_media_info * /*mediaInfo*/,
			     struct mbuf_raw_video_frame * /*frame*/,
			     const void * /*frameUserdata*/,
			     size_t /*frameUserdataLen*/) override
	{
		if (textureWidth > 0) {
			mLoadedTextureWidth = textureWidth;
			mLoadedTextureHeight = textureHeight;
			mLoadCallCount++;
		}
		return 0;
	}

	unsigned int mLoadedTextureWidth = 0;
	unsigned int mLoadedTextureHeight = 0;
	int mLoadCallCount = 0;
};


/* Extends VideoRendererTrackingListener to capture histogram data from the
 * renderVideoOverlay callback.  The base class returns -ENOSYS from
 * renderVideoOverlay, which prevents GlVideoRenderer from ever calling the
 * overlay path (mRenderVideoOverlay stays false).  Returning 0 here instead
 * makes the constructor's probe call succeed, setting mRenderVideoOverlay =
 * true, so the real render() path computes histograms and delivers them in
 * frameExtra.histogram_len[]. */
class HistogramCapturingRendererListener
		: public VideoRendererTrackingListener {
public:
	int
	renderVideoOverlay(IPdraw * /*p*/,
			   IPdraw::IVideoRenderer * /*r*/,
			   const struct pdraw_rect * /*renderPos*/,
			   const struct pdraw_rect * /*contentPos*/,
			   const float * /*viewMat*/,
			   const float * /*projMat*/,
			   const struct pdraw_media_info * /*mediaInfo*/,
			   struct vmeta_frame * /*frameMeta*/,
			   const struct pdraw_video_frame_extra *extra) override
	{
		mOverlayCallCount++;
		if (extra) {
			for (int i = 0; i < PDRAW_HISTOGRAM_CHANNEL_MAX; i++)
				mHistogramLen[i] = extra->histogram_len[i];
		}
		return 0;
	}

	int mOverlayCallCount = 0;
	size_t mHistogramLen[PDRAW_HISTOGRAM_CHANNEL_MAX] = {};
};


/* Captures video.session_meta->friendly_name from every renderVideoOverlay()
 * call, to observe GlVideoRenderer::onChannelSessionMetaUpdate()'s effect on
 * mMediaInfo.video.session_meta: there is no public getter for it, but
 * render() always points the overlay callback's mediaInfo argument at the
 * live mMediaInfo (pdraw_renderer_video_gl.cpp:2280/2546), so this is the
 * only observable surface. Same "return 0, not -ENOSYS" requirement as
 * HistogramCapturingRendererListener to activate the overlay path at all. */
class SessionMetaCapturingRendererListener
		: public VideoRendererTrackingListener {
public:
	int renderVideoOverlay(
		IPdraw * /*p*/,
		IPdraw::IVideoRenderer * /*r*/,
		const struct pdraw_rect * /*renderPos*/,
		const struct pdraw_rect * /*contentPos*/,
		const float * /*viewMat*/,
		const float * /*projMat*/,
		const struct pdraw_media_info *mediaInfo,
		struct vmeta_frame * /*frameMeta*/,
		const struct pdraw_video_frame_extra * /*extra*/) override
	{
		mOverlayCallCount++;
		if ((mediaInfo != nullptr) &&
		    (mediaInfo->video.session_meta != nullptr)) {
			memcpy(mFriendlyName,
			       mediaInfo->video.session_meta->friendly_name,
			       sizeof(mFriendlyName));
		}
		return 0;
	}

	int mOverlayCallCount = 0;
	char mFriendlyName[40] = {};
};


/* Combines ExtTexCapturingRendererListener (activates the ext-texture
 * loading path) and HistogramCapturingRendererListener (captures histogram
 * data from the overlay callback) into one listener. Needed because
 * GlVideoRenderer::renderExternalVideoFrame() always renders ext-texture
 * frames as format &vdef_rgb (pdraw_renderer_video_gl.cpp:1690), which
 * GlVideo::getProgram() maps to Program::NOCONV (pdraw_gl_video.cpp:
 * 1122-1123) regardless of the raw video source's own declared format --
 * i.e. this is the only way to reach the NOCONV branch of renderPadding()/
 * computeHistograms()/renderBlur() at all, and it requires both listener
 * behaviors active at once to also observe histogram output. */
class ExtTexHistogramRendererListener : public VideoRendererTrackingListener {
public:
	int loadVideoTexture(IPdraw * /*p*/,
			     IPdraw::IVideoRenderer * /*r*/,
			     unsigned int textureWidth,
			     unsigned int /*textureHeight*/,
			     const struct pdraw_media_info * /*mediaInfo*/,
			     struct mbuf_raw_video_frame * /*frame*/,
			     const void * /*frameUserdata*/,
			     size_t /*frameUserdataLen*/) override
	{
		if (textureWidth > 0)
			mLoadCallCount++;
		return 0;
	}

	int
	renderVideoOverlay(IPdraw * /*p*/,
			   IPdraw::IVideoRenderer * /*r*/,
			   const struct pdraw_rect * /*renderPos*/,
			   const struct pdraw_rect * /*contentPos*/,
			   const float * /*viewMat*/,
			   const float * /*projMat*/,
			   const struct pdraw_media_info * /*mediaInfo*/,
			   struct vmeta_frame * /*frameMeta*/,
			   const struct pdraw_video_frame_extra *extra) override
	{
		mOverlayCallCount++;
		if (extra) {
			for (int i = 0; i < PDRAW_HISTOGRAM_CHANNEL_MAX; i++)
				mHistogramLen[i] = extra->histogram_len[i];
		}
		return 0;
	}

	int mLoadCallCount = 0;
	int mOverlayCallCount = 0;
	size_t mHistogramLen[PDRAW_HISTOGRAM_CHANNEL_MAX] = {};
};


/* Minimal RAII offscreen (1x1 pbuffer) GL ES 3 context bound for the
 * lifetime of the object -- see the file header comment for why libpdraw
 * itself needs one already current, and what this mirrors. Simplified
 * compared to its model (no refcounting, no separate bind()/unbind()):
 * this test only ever uses one context on one thread. */
class TestGlContext {
public:
	TestGlContext()
	{
		/* These calls are the environment probe: no EGL/GL driver
		 * at all in this environment (headless CI, no Mesa software
		 * rasterizer installed, ...) is not a bug, so don't
		 * CU_ASSERT_FATAL here -- just leave mDisplay/mContext at
		 * their EGL_NO_* defaults and let isValid() report false.
		 * Everything from here on assumes EGL is genuinely available,
		 * so any further failure IS a real bug worth flagging. */
		setenv("LIBGL_ALWAYS_SOFTWARE", "1", 0);

		/* Prefer Mesa's surfaceless platform: software rendering
		 * without an X server or DRM device, unlike the legacy
		 * eglGetDisplay(EGL_DEFAULT_DISPLAY) fallback below which
		 * headless CI containers usually can't satisfy. */
		typedef EGLDisplay (*GetPlatformDisplayExtProc)(
			EGLenum platform,
			void *native_display,
			const EGLint *attrib_list);
		auto getPlatformDisplayExt =
			reinterpret_cast<GetPlatformDisplayExtProc>(
				eglGetProcAddress("eglGetPlatformDisplayEXT"));
		if (getPlatformDisplayExt != nullptr) {
#	ifndef EGL_PLATFORM_SURFACELESS_MESA
#		define EGL_PLATFORM_SURFACELESS_MESA 0x31DD
#	endif
			mDisplay = getPlatformDisplayExt(
				EGL_PLATFORM_SURFACELESS_MESA,
				EGL_DEFAULT_DISPLAY,
				nullptr);
		}
		if (mDisplay != EGL_NO_DISPLAY) {
			ULOGI("TestGlContext: got EGL display via the "
			      "surfaceless platform");
		} else {
			mDisplay = eglGetDisplay(EGL_DEFAULT_DISPLAY);
			if (mDisplay != EGL_NO_DISPLAY) {
				ULOGI("TestGlContext: got EGL display via "
				      "the legacy eglGetDisplay() fallback");
			}
		}
		if (mDisplay == EGL_NO_DISPLAY) {
			ULOGW("TestGlContext: no EGL display available in "
			      "this environment, GL tests will be skipped");
			return;
		}

		EGLint major = 0;
		EGLint minor = 0;
		EGLBoolean res = eglInitialize(mDisplay, &major, &minor);
		if (res != EGL_TRUE) {
			ULOGW("TestGlContext: eglInitialize() failed, GL "
			      "tests will be skipped");
			mDisplay = EGL_NO_DISPLAY;
			return;
		}
		ULOGI("TestGlContext: EGL initialized (version %d.%d)",
		      major,
		      minor);

		static const EGLint configAttr[] = {
			EGL_RED_SIZE,
			8,
			EGL_GREEN_SIZE,
			8,
			EGL_BLUE_SIZE,
			8,
			EGL_ALPHA_SIZE,
			8,
			EGL_SURFACE_TYPE,
			EGL_PBUFFER_BIT,
			EGL_RENDERABLE_TYPE,
			EGL_OPENGL_ES3_BIT,
			EGL_NONE,
		};
		EGLConfig config;
		EGLint numConfigs = 0;
		res = eglChooseConfig(
			mDisplay, configAttr, &config, 1, &numConfigs);
		CU_ASSERT_FATAL(res == EGL_TRUE);
		CU_ASSERT_FATAL(numConfigs > 0);

		res = eglBindAPI(EGL_OPENGL_ES_API);
		CU_ASSERT_FATAL(res == EGL_TRUE);

		static const EGLint contextAttr[] = {
			EGL_CONTEXT_CLIENT_VERSION,
			3,
			EGL_NONE,
		};
		mContext = eglCreateContext(
			mDisplay, config, EGL_NO_CONTEXT, contextAttr);
		CU_ASSERT_FATAL(mContext != EGL_NO_CONTEXT);

		static const EGLint surfaceAttr[] = {
			EGL_WIDTH,
			1,
			EGL_HEIGHT,
			1,
			EGL_NONE,
		};
		mSurface =
			eglCreatePbufferSurface(mDisplay, config, surfaceAttr);
		CU_ASSERT_FATAL(mSurface != EGL_NO_SURFACE);

		res = eglMakeCurrent(mDisplay, mSurface, mSurface, mContext);
		CU_ASSERT_FATAL(res == EGL_TRUE);
	}

	~TestGlContext()
	{
		eglMakeCurrent(mDisplay,
			       EGL_NO_SURFACE,
			       EGL_NO_SURFACE,
			       EGL_NO_CONTEXT);
		if (mSurface != EGL_NO_SURFACE)
			eglDestroySurface(mDisplay, mSurface);
		if (mContext != EGL_NO_CONTEXT)
			eglDestroyContext(mDisplay, mContext);
		if (mDisplay != EGL_NO_DISPLAY)
			eglTerminate(mDisplay);
	}

	TestGlContext(const TestGlContext &) = delete;
	TestGlContext &operator=(const TestGlContext &) = delete;

	/* False when the constructor's CU_ASSERT_FATAL bailed out early (no
	 * working EGL/GL driver in this environment) -- the destructor
	 * already tolerates a partially-constructed instance (each cleanup
	 * step is itself guarded by a != EGL_NO_* check), so this is safe to
	 * query and let the object be destroyed normally. */
	bool isValid() const
	{
		return mContext != EGL_NO_CONTEXT;
	}

private:
	EGLDisplay mDisplay = EGL_NO_DISPLAY;
	EGLContext mContext = EGL_NO_CONTEXT;
	EGLSurface mSurface = EGL_NO_SURFACE;
};


/* Declares `var` as a TestGlContext and, if EGL/GL isn't available in this
 * environment, CU_PASSes and returns from the calling test -- factors out
 * the declare+check+skip boilerplate needed at the top of every test in
 * this file. Must be used as a standalone statement (declares a variable
 * meant to stay in scope for the rest of the function), not inside a
 * single-statement if/else without braces. */
#	define REQUIRE_GL_CONTEXT(var)                                        \
		TestGlContext var;                                             \
		if (!(var).isValid()) {                                        \
			CU_PASS("EGL/GL not available in this environment");   \
			return;                                                \
		}


/* RAII helper: sets the PDRAW_VIDEO_RENDERER_DBG_FLAGS environment variable
 * for the lifetime of the object, restoring (or unsetting) whatever value
 * was there before on destruction.
 *
 * This matters because GlVideoRenderer::setParams() calls getenv() on this
 * exact variable on *every* call (pdraw_renderer_video_gl.cpp:2715), both
 * from its own constructor and from the public setParams() API -- and
 * setenv() changes process-wide state. Without restoring the previous value,
 * a test using this would leak into every other test_pipeline_renderer_
 * video.cpp test that creates a renderer afterward, in this same CUnit
 * binary. */
class ScopedDbgFlagsEnv {
public:
	explicit ScopedDbgFlagsEnv(const char *value)
	{
		const char *prev = getenv("PDRAW_VIDEO_RENDERER_DBG_FLAGS");
		if (prev != nullptr) {
			mHadPrev = true;
			mPrevValue = prev;
		}
		setenv("PDRAW_VIDEO_RENDERER_DBG_FLAGS", value, 1);
	}

	~ScopedDbgFlagsEnv()
	{
		if (mHadPrev)
			setenv("PDRAW_VIDEO_RENDERER_DBG_FLAGS",
			       mPrevValue.c_str(),
			       1);
		else
			unsetenv("PDRAW_VIDEO_RENDERER_DBG_FLAGS");
	}

	ScopedDbgFlagsEnv(const ScopedDbgFlagsEnv &) = delete;
	ScopedDbgFlagsEnv &operator=(const ScopedDbgFlagsEnv &) = delete;

private:
	bool mHadPrev = false;
	std::string mPrevValue;
};

} /* anonymous namespace */


/* Builds a flat-grey frame of any renderer-supported raw format (no asset
 * file, self-contained): content is irrelevant here, only that a real frame
 * with correctly-sized planes reaches the renderer's texture upload /
 * format-specific shader path.  Callers pass the vdef format constant and
 * its nominal bit depth; vdef_calc_raw_frame_size /
 * vdef_get_raw_frame_plane_count derive strides and plane count, so the same
 * body works for I420 (3 planes), NV12 (2 planes), and GRAY (1 plane).
 * metadata defaults to nullptr (no per-frame vmeta_frame attached); callers
 * needing one (e.g. for GlVideoRenderer::setNormalization()) pass it
 * explicitly -- must be set before finalize(), same order already used by
 * test_pipeline_muxer_record.cpp. */
static struct mbuf_raw_video_frame *
makeRawVideoFrame(const struct vdef_raw_format &fmt,
		  uint8_t bitDepth,
		  uint32_t width,
		  uint32_t height,
		  uint64_t timestamp,
		  unsigned int index,
		  struct vmeta_frame *metadata = nullptr,
		  uint32_t timescale = 1000000,
		  uint32_t sarWidth = 1,
		  uint32_t sarHeight = 1)
{
	struct vdef_dim resolution = {width, height};
	size_t planeStride[VDEF_RAW_MAX_PLANE_COUNT] = {};
	size_t planeSize[VDEF_RAW_MAX_PLANE_COUNT] = {};
	int ret = vdef_calc_raw_frame_size(&fmt,
					   &resolution,
					   planeStride,
					   nullptr,
					   nullptr,
					   nullptr,
					   planeSize,
					   nullptr);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	unsigned int planeCount = vdef_get_raw_frame_plane_count(&fmt);
	CU_ASSERT_TRUE_FATAL(planeCount >= 1u);

	struct vdef_raw_frame frameInfo = {};
	frameInfo.format = fmt;
	frameInfo.info.timescale = timescale;
	frameInfo.info.timestamp = timestamp;
	frameInfo.info.index = index;
	frameInfo.info.resolution.width = width;
	frameInfo.info.resolution.height = height;
	/* GlVideoRenderer::loadVideoFrame() rejects a frame as having
	 * "invalid frame dimensions" if sar is null, on top of resolution
	 * (square pixels: 1:1). */
	frameInfo.info.sar.width = sarWidth;
	frameInfo.info.sar.height = sarHeight;
	frameInfo.info.bit_depth = bitDepth;
	for (unsigned int p = 0; p < planeCount; p++)
		frameInfo.plane_stride[p] = planeStride[p];

	struct mbuf_raw_video_frame *frame = nullptr;
	ret = mbuf_raw_video_frame_new(&frameInfo, &frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	for (unsigned int p = 0; p < planeCount; p++) {
		struct mbuf_mem *mem = nullptr;
		ret = mbuf_mem_generic_new(planeSize[p], &mem);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
		uint8_t *data = nullptr;
		size_t capacity = 0;
		ret = mbuf_mem_get_data(mem, (void **)&data, &capacity);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
		memset(data, 0x80, capacity);
		ret = mbuf_raw_video_frame_set_plane(
			frame, p, mem, 0, planeSize[p]);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
		mbuf_mem_unref(mem);
	}

	if (metadata != nullptr) {
		ret = mbuf_raw_video_frame_set_metadata(frame, metadata);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
	}

	ret = mbuf_raw_video_frame_finalize(frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	return frame;
}


/* Shared by every test below: creates a real raw video source, waits for
 * its output media, then a real GlVideoRenderer on that media (with
 * glContext already bound -- required by its constructor, see the file
 * header comment), and waits for the renderer's own onVideoRendererMediaAdded.
 * Does not push a frame or render: callers needing an actual rendered frame
 * do that themselves right after this returns.
 * rawFmt/bitDepth default to I420/8 so format-agnostic tests need not pass
 * them; format-specific tests (NV12, GRAY) pass them explicitly.
 * sessionMeta defaults to nullptr (zero-initialized session_meta, as before);
 * tests needing a specific session_meta at source-creation time (e.g.
 * camera_spectrum for GlVideoRenderer::setNormalization()) pass it. */
static void setupRendererWithSource(
	TestPompLoop &loop,
	IPdraw *session,
	uint32_t width,
	uint32_t height,
	const struct pdraw_rect &renderPos,
	const struct pdraw_video_renderer_params &rendererParams,
	MediaTrackingListener &mediaListener,
	VideoRendererTrackingListener &rendererListener,
	IPdraw::IRawVideoSource **outSource,
	IPdraw::IVideoRenderer **outRenderer,
	const struct vdef_raw_format &rawFmt = vdef_i420,
	uint8_t bitDepth = 8,
	IPdraw::IRawVideoSource::Listener *srcListener = nullptr,
	const struct vmeta_session *sessionMeta = nullptr)
{
	struct pdraw_video_source_params sourceParams = {};
	sourceParams.playback_type = PDRAW_PLAYBACK_TYPE_LIVE;
	sourceParams.video.format = VDEF_FRAME_TYPE_RAW;
	sourceParams.video.raw.format = rawFmt;
	sourceParams.video.raw.info.resolution.width = width;
	sourceParams.video.raw.info.resolution.height = height;
	sourceParams.video.raw.info.bit_depth = bitDepth;
	sourceParams.video.raw.info.framerate.num = 30;
	sourceParams.video.raw.info.framerate.den = 1;
	if (sessionMeta != nullptr)
		sourceParams.session_meta = *sessionMeta;

	auto *sl = srcListener
			   ? srcListener
			   : static_cast<IPdraw::IRawVideoSource::Listener *>(
				     &g_stub_raw_video_source_listener);
	int ret = session->createRawVideoSource(&sourceParams, sl, outSource);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(*outSource);

	bool gotRawMedia = loop.pumpUntil(
		[&mediaListener]() {
			return mediaListener.findRawVideoMedia() != nullptr;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotRawMedia);
	unsigned int rawMediaId = mediaListener.findRawVideoMedia()->id;

	ret = session->createVideoRenderer(rawMediaId,
					   &renderPos,
					   &rendererParams,
					   &rendererListener,
					   outRenderer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(*outRenderer);

	bool gotRendererMedia = loop.pumpUntil(
		[&rendererListener]() {
			return rendererListener.mGotMediaAdded;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotRendererMedia);
}


/* Pushes one flat-grey frame of the given format into source's queue and
 * waits for the renderer to signal it is ready to render it (default
 * scheduling_mode is ASAP (0): GlVideoRenderer::queueEventCb() calls
 * onVideoRenderReady() synchronously, on the loop thread, as soon as the
 * frame is queued -- no real-time wait needed).
 * fmt/bitDepth default to I420/8; format-specific tests pass them explicitly.
 * metadata defaults to nullptr; see makeRawVideoFrame(). */
static void
pushFrameAndWaitRenderReady(TestPompLoop &loop,
			    IPdraw::IRawVideoSource *source,
			    VideoRendererTrackingListener &rendererListener,
			    uint32_t width,
			    uint32_t height,
			    uint64_t timestamp,
			    unsigned int index,
			    const struct vdef_raw_format &fmt = vdef_i420,
			    uint8_t bitDepth = 8,
			    struct vmeta_frame *metadata = nullptr)
{
	struct mbuf_raw_video_frame_queue *inQueue = source->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(inQueue);
	struct mbuf_raw_video_frame *frame = makeRawVideoFrame(
		fmt, bitDepth, width, height, timestamp, index, metadata);
	int ret = mbuf_raw_video_frame_queue_push(inQueue, frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	mbuf_raw_video_frame_unref(frame);

	int countBefore = rendererListener.mRenderReadyCount.load();
	bool gotRenderReady = loop.pumpUntil(
		[&]() {
			return rendererListener.mRenderReadyCount.load() >
			       countBefore;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotRenderReady);
}


/* End-to-end render of one real frame against a real (offscreen) GL context
 * for the given raw format -- the gap left open by
 * test_api_renderer_video.cpp's testCxxCreateValid().  Used by the per-format
 * test functions below. */
static void runRenderRealFrameTest(const struct vdef_raw_format &fmt,
				   uint8_t bitDepth)
{
	static constexpr uint32_t kWidth = 64;
	static constexpr uint32_t kHeight = 64;

	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	/* Must be current before createVideoRenderer(): its constructor
	 * calls GlVideoRenderer::setup() synchronously, which queries
	 * GL_FRAMEBUFFER_BINDING (see the file header comment). */
	REQUIRE_GL_CONTEXT(glContext);

	struct pdraw_rect renderPos = {0, 0, kWidth, kHeight};
	struct pdraw_video_renderer_params rendererParams = {};
	VideoRendererTrackingListener rendererListener;
	IPdraw::IRawVideoSource *source = nullptr;
	IPdraw::IVideoRenderer *renderer = nullptr;
	setupRendererWithSource(loop,
				session,
				kWidth,
				kHeight,
				renderPos,
				rendererParams,
				mediaListener,
				rendererListener,
				&source,
				&renderer,
				fmt,
				bitDepth);
	auto rendererOwner = std::unique_ptr<IPdraw::IVideoRenderer>(renderer);
	auto sourceOwner = std::unique_ptr<IPdraw::IRawVideoSource>(source);

	pushFrameAndWaitRenderReady(loop,
				    source,
				    rendererListener,
				    kWidth,
				    kHeight,
				    0,
				    0,
				    fmt,
				    bitDepth);

	struct pdraw_rect contentPos = {};
	int ret = renderer->render(&contentPos);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_TRUE(contentPos.width > 0);
	CU_ASSERT_TRUE(contentPos.height > 0);

	CU_ASSERT_EQUAL(glGetError(), (GLenum)GL_NO_ERROR);

	/* Stop the session (and wait for it) BEFORE resetting the owners
	 * below, so that Session::asyncElementDelete() destroys the
	 * VideoRenderer element -- and thus runs VideoRendererWrapper::
	 * clearElement() -- while rendererOwner is still alive. Resetting the
	 * owners first (as this test used to do) would run ~ElementWrapper()
	 * instead and the override would never execute. glContext (still in
	 * scope until function return) stays bound throughout, so this
	 * reordering does not affect the GL-context constraint below. */
	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);

	/* VideoRendererWrapper has no getRenderer()-style accessor (mRenderer
	 * is private with no getter), but ElementWrapper::clearElement() nulls
	 * the base mElement in the same override call, right before mRenderer
	 * -- getElement() returning null is our proof that the override ran. */
	CU_ASSERT_PTR_NULL(
		static_cast<VideoRendererWrapper *>(renderer)->getElement());

	/* renderer's destructor issues real GL calls (releases textures/
	 * shaders/framebuffers): must run while glContext is still bound. */
	rendererOwner.reset();
	sourceOwner.reset();
}


static void testCxxVideoRendererRendersRealFrame()
{
	runRenderRealFrameTest(vdef_i420, 8);
}

/* NV12 (2-plane semi-planar YUV 4:2:0, 8-bit) -- exercises the NV12 shader
 * path in pdraw_gl_video.cpp (vdef_nv12 branch). */
static void testCxxVideoRendererRendersRealFrameNV12()
{
	runRenderRealFrameTest(vdef_nv12, 8);
}

/* GRAY (1-plane 8-bit luma only) -- exercises the GRAY shader path in
 * pdraw_gl_video.cpp (vdef_gray branch).
 * Note: vdef_gray16 (16-bit gray) is NOT in the renderer's
 * getSupportedFormats() list; vdef_gray (8-bit) is the supported gray format.
 */
static void testCxxVideoRendererRendersRealFrameGray()
{
	runRenderRealFrameTest(vdef_gray, 8);
}

/* RAW16 (1-plane 16-bit opaque) -- exercises the RAW16 shader path in
 * pdraw_gl_video.cpp (vdef_raw16 branch).  Structurally identical to
 * vdef_gray16 (both PACKED, 16-bit, 1 plane) but with a RAW rather than GRAY
 * color model -- hence a distinct shader path. */
static void testCxxVideoRendererRendersRealFrameRaw16()
{
	runRenderRealFrameTest(vdef_raw16, 16);
}


/* I420_10_16LE (3-plane 10-bit planar YUV 4:2:0 in 16-bit words) -- exercises
 * Program::YUV_TO_RGB_PLANAR_10_16LE (pdraw_gl_video.cpp:3106). */
static void testCxxVideoRendererRendersRealFrameI420_10_16LE()
{
	runRenderRealFrameTest(vdef_i420_10_16le, 10);
}


/* NV12_10_16LE_HIGH (2-plane 10-bit semi-planar YUV 4:2:0 in 16-bit words) --
 * exercises Program::YUV_TO_RGB_SEMIPLANAR_10_16LE_HIGH
 * (pdraw_gl_video.cpp:3158). */
static void testCxxVideoRendererRendersRealFrameNV12_10_16LE_HIGH()
{
	runRenderRealFrameTest(vdef_nv12_10_16le_high, 10);
}


/* RAW32 (1-plane 32-bit grayscale/raw) -- exercises
 * Program::GRAY32_TO_RGB_PLANAR (pdraw_gl_video.cpp:3221). */
static void testCxxVideoRendererRendersRealFrameRaw32()
{
	runRenderRealFrameTest(vdef_raw32, 32);
}


/* Verifies that enable_histograms = 1 causes histogram data to be computed
 * and delivered in the renderVideoOverlay frameExtra.  Uses I420 (grey frame):
 * Y=U=V=0x80 → after YUV-to-RGB, R=G=B≈0.5, so all four channels (RED,
 * GREEN, BLUE, LUMA) should produce a valid 256-bin histogram on the first
 * render() call.
 *
 * Constraint: the overlay callback must return 0 (not -ENOSYS) so that
 * GlVideoRenderer sets mRenderVideoOverlay=true during construction (see the
 * "probe" call at constructor time).  HistogramCapturingRendererListener
 * satisfies this while also recording histogram_len[] from frameExtra. */
static void testCxxVideoRendererHistogramComputed()
{
	static constexpr uint32_t kWidth = 64;
	static constexpr uint32_t kHeight = 64;

	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	REQUIRE_GL_CONTEXT(glContext);

	struct pdraw_rect renderPos = {0, 0, kWidth, kHeight};
	struct pdraw_video_renderer_params rendererParams = {};
	rendererParams.enable_histograms = 1;

	HistogramCapturingRendererListener rendererListener;
	IPdraw::IRawVideoSource *source = nullptr;
	IPdraw::IVideoRenderer *renderer = nullptr;
	setupRendererWithSource(loop,
				session,
				kWidth,
				kHeight,
				renderPos,
				rendererParams,
				mediaListener,
				rendererListener,
				&source,
				&renderer);
	auto rendererOwner = std::unique_ptr<IPdraw::IVideoRenderer>(renderer);
	auto sourceOwner = std::unique_ptr<IPdraw::IRawVideoSource>(source);

	pushFrameAndWaitRenderReady(
		loop, source, rendererListener, kWidth, kHeight, 0, 0);

	struct pdraw_rect contentPos = {};
	int ret = renderer->render(&contentPos);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_TRUE(contentPos.width > 0);

	/* Overlay was invoked and all four histogram channels were computed. */
	CU_ASSERT_TRUE(rendererListener.mOverlayCallCount >= 1);
	CU_ASSERT_EQUAL(
		rendererListener.mHistogramLen[PDRAW_HISTOGRAM_CHANNEL_LUMA],
		256u);
	CU_ASSERT_EQUAL(
		rendererListener.mHistogramLen[PDRAW_HISTOGRAM_CHANNEL_RED],
		256u);
	CU_ASSERT_EQUAL(
		rendererListener.mHistogramLen[PDRAW_HISTOGRAM_CHANNEL_GREEN],
		256u);
	CU_ASSERT_EQUAL(
		rendererListener.mHistogramLen[PDRAW_HISTOGRAM_CHANNEL_BLUE],
		256u);

	CU_ASSERT_EQUAL(glGetError(), (GLenum)GL_NO_ERROR);

	rendererOwner.reset();
	sourceOwner.reset();

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
}


/* drain() on the source after one frame has been pushed and render-readied.
 * At the time drain() is called, queueEventCb has already moved the frame
 * from the queue into mNextFrame, so the queue is empty and onChannelDrain
 * fires asyncDrainDone() immediately — the drained callback reaches the
 * source listener on the very next loop iteration. */
static void testCxxVideoRendererDrainFiresCallback()
{
	static constexpr uint32_t kWidth = 64;
	static constexpr uint32_t kHeight = 64;

	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	REQUIRE_GL_CONTEXT(glContext);

	struct pdraw_rect renderPos = {0, 0, kWidth, kHeight};
	struct pdraw_video_renderer_params rendererParams = {};
	VideoRendererTrackingListener rendererListener;
	TrackingRawVideoSourceListener sourceListener;
	IPdraw::IRawVideoSource *source = nullptr;
	IPdraw::IVideoRenderer *renderer = nullptr;
	setupRendererWithSource(loop,
				session,
				kWidth,
				kHeight,
				renderPos,
				rendererParams,
				mediaListener,
				rendererListener,
				&source,
				&renderer,
				vdef_i420,
				8,
				&sourceListener);
	auto rendererOwner = std::unique_ptr<IPdraw::IVideoRenderer>(renderer);
	auto sourceOwner = std::unique_ptr<IPdraw::IRawVideoSource>(source);

	pushFrameAndWaitRenderReady(
		loop, source, rendererListener, kWidth, kHeight, 0, 0);

	int ret = source->drain();
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	bool gotDrained = loop.pumpUntil(
		[&sourceListener]() { return sourceListener.mGotDrained; },
		5000);
	CU_ASSERT_TRUE(gotDrained);

	CU_ASSERT_EQUAL(glGetError(), (GLenum)GL_NO_ERROR);

	rendererOwner.reset();
	sourceOwner.reset();

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
}


/* flush() on the source after one frame has been pushed, render-readied and
 * actually rendered.  flush() discards queued frames and clears mLoadedFrame /
 * mNextFrame inside the renderer; the flushed callback fires once
 * asyncFlushDone() reaches the source listener.
 *
 * Because flush() clears the mbuf reference but leaves the GL textures intact,
 * a subsequent render() still shows the last uploaded frame (freeze-on-last-
 * frame, the same behaviour as after removeInputMedia -- confirmed by reading
 * renderVideoFrame() which uses mLoadedFrame.info for geometry but reads pixel
 * data from already-uploaded GL textures, not from the mbuf pointer). */
static void testCxxVideoRendererFlushFiresCallbackAndFreezesFrame()
{
	static constexpr uint32_t kWidth = 64;
	static constexpr uint32_t kHeight = 64;

	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	REQUIRE_GL_CONTEXT(glContext);

	struct pdraw_rect renderPos = {0, 0, kWidth, kHeight};
	struct pdraw_video_renderer_params rendererParams = {};
	VideoRendererTrackingListener rendererListener;
	TrackingRawVideoSourceListener sourceListener;
	IPdraw::IRawVideoSource *source = nullptr;
	IPdraw::IVideoRenderer *renderer = nullptr;
	setupRendererWithSource(loop,
				session,
				kWidth,
				kHeight,
				renderPos,
				rendererParams,
				mediaListener,
				rendererListener,
				&source,
				&renderer,
				vdef_i420,
				8,
				&sourceListener);
	auto rendererOwner = std::unique_ptr<IPdraw::IVideoRenderer>(renderer);
	auto sourceOwner = std::unique_ptr<IPdraw::IRawVideoSource>(source);

	pushFrameAndWaitRenderReady(
		loop, source, rendererListener, kWidth, kHeight, 0, 0);

	/* Render once to load the frame into GL textures (mFrameLoaded = true).
	 */
	struct pdraw_rect contentPosBefore = {};
	int ret = renderer->render(&contentPosBefore);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_TRUE_FATAL(contentPosBefore.width > 0);

	ret = source->flush();
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	bool gotFlushed = loop.pumpUntil(
		[&sourceListener]() { return sourceListener.mGotFlushed; },
		5000);
	CU_ASSERT_TRUE(gotFlushed);

	/* After flush the mbuf is released but GL textures remain; render()
	 * must succeed and return the same content dimensions
	 * (freeze-on-last-frame). */
	struct pdraw_rect contentPosAfter = {};
	ret = renderer->render(&contentPosAfter);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(contentPosAfter.width, contentPosBefore.width);
	CU_ASSERT_EQUAL(contentPosAfter.height, contentPosBefore.height);

	CU_ASSERT_EQUAL(glGetError(), (GLenum)GL_NO_ERROR);

	rendererOwner.reset();
	sourceOwner.reset();

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
}


/* Exercises the external texture loading code path:
 *   video_texture_width + video_texture_dar_* → setupExtTexture() computes
 *   the FBO dimensions → startExtLoad() creates the RGBA FBO texture →
 *   loadVideoTexture() callback is called with those dimensions →
 *   renderExternalVideoFrame() renders via NOCONV program.
 *
 * Source: 64×64 I420 (SAR 1:1).
 * Params: video_texture_width=64, DAR=2:1.
 * Expected texture: width=64, height=(64×1 + 2/2)/2 = 32 (rounded up to
 * next even: 32, already even).  Formula from setupExtTexture() "custom DAR
 * with custom texture width" branch. */
static void testCxxVideoRendererExtTextureWithDAR()
{
	static constexpr uint32_t kWidth = 64;
	static constexpr uint32_t kHeight = 64;

	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	REQUIRE_GL_CONTEXT(glContext);

	struct pdraw_rect renderPos = {0, 0, kWidth, kHeight};
	struct pdraw_video_renderer_params rendererParams = {};
	rendererParams.video_texture_width = 64;
	rendererParams.video_texture_dar_width = 2;
	rendererParams.video_texture_dar_height = 1;

	ExtTexCapturingRendererListener rendererListener;
	IPdraw::IRawVideoSource *source = nullptr;
	IPdraw::IVideoRenderer *renderer = nullptr;
	setupRendererWithSource(loop,
				session,
				kWidth,
				kHeight,
				renderPos,
				rendererParams,
				mediaListener,
				rendererListener,
				&source,
				&renderer);
	auto rendererOwner = std::unique_ptr<IPdraw::IVideoRenderer>(renderer);
	auto sourceOwner = std::unique_ptr<IPdraw::IRawVideoSource>(source);

	pushFrameAndWaitRenderReady(
		loop, source, rendererListener, kWidth, kHeight, 0, 0);

	struct pdraw_rect contentPos = {};
	int ret = renderer->render(&contentPos);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_TRUE(contentPos.width > 0);

	/* loadVideoTexture was invoked with the DAR-computed dimensions. */
	CU_ASSERT_TRUE(rendererListener.mLoadCallCount >= 1);
	CU_ASSERT_EQUAL(rendererListener.mLoadedTextureWidth, 64u);
	CU_ASSERT_EQUAL(rendererListener.mLoadedTextureHeight, 32u);

	CU_ASSERT_EQUAL(glGetError(), (GLenum)GL_NO_ERROR);

	rendererOwner.reset();
	sourceOwner.reset();

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
}


/* Exercises PDRAW_VIDEO_RENDERER_SCHEDULING_MODE_ADAPTIVE (the default mode
 * used in the production app) end-to-end with two frames.
 *
 * Frame 1 (T=0):
 *   First call to getNextFrameDelay() has mSchedLastInputTimestamp=UINT64_MAX
 *   → epsilon=0, timingError=0, addedTimingError=0 → ON-TIME → delay=0
 *   → onVideoRenderReady fires synchronously (same as ASAP for first frame).
 *
 * Frame 2 (T=33333 µs, ~30 fps inter-frame):
 *   timingError ≈ +33 ms (frame is early vs. wall-clock-based schedule) plus
 *   addedTimingError ≈ +16 ms (initial-buffering 50% bonus) → EARLY → delay
 *   ≈ 50 ms → timer armed.  After ~50 ms the timer fires, curTime has caught
 *   up → frame is now LATE → delay=0 → onVideoRenderReady fires → render()
 *   dequeues and renders the frame.
 *
 * This covers: ADAPTIVE branch in getNextFrameDelay, initial-buffering state
 * machine, EARLY/LATE sub-branches, and the timer-based render-ready path
 * that ASAP never takes.  The 15-second pumpUntil timeout easily accommodates
 * the ~50 ms timer wait. */
static void testCxxVideoRendererAdaptiveSchedulingRendersFrames()
{
	static constexpr uint32_t kWidth = 64;
	static constexpr uint32_t kHeight = 64;
	/* 30 fps inter-frame in µs (timescale=1000000) */
	static constexpr uint64_t kFrameIntervalUs = 33333;

	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	REQUIRE_GL_CONTEXT(glContext);

	struct pdraw_rect renderPos = {0, 0, kWidth, kHeight};
	struct pdraw_video_renderer_params rendererParams = {};
	rendererParams.scheduling_mode =
		PDRAW_VIDEO_RENDERER_SCHEDULING_MODE_ADAPTIVE;

	VideoRendererTrackingListener rendererListener;
	IPdraw::IRawVideoSource *source = nullptr;
	IPdraw::IVideoRenderer *renderer = nullptr;
	setupRendererWithSource(loop,
				session,
				kWidth,
				kHeight,
				renderPos,
				rendererParams,
				mediaListener,
				rendererListener,
				&source,
				&renderer);
	auto rendererOwner = std::unique_ptr<IPdraw::IVideoRenderer>(renderer);
	auto sourceOwner = std::unique_ptr<IPdraw::IRawVideoSource>(source);

	/* Frame 1: T=0, delay=0 in ADAPTIVE → render-ready fires synchronously
	 */
	pushFrameAndWaitRenderReady(
		loop, source, rendererListener, kWidth, kHeight, 0, 0);

	struct pdraw_rect contentPos1 = {};
	int ret = renderer->render(&contentPos1);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_TRUE_FATAL(contentPos1.width > 0);

	/* Frame 2: T=33333 µs, EARLY+initial-buffering → ~50 ms timer before
	 * render-ready.  pumpUntil (15 s timeout) absorbs the wait. */
	pushFrameAndWaitRenderReady(loop,
				    source,
				    rendererListener,
				    kWidth,
				    kHeight,
				    kFrameIntervalUs,
				    1);

	struct pdraw_rect contentPos2 = {};
	ret = renderer->render(&contentPos2);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_TRUE(contentPos2.width > 0);

	CU_ASSERT_EQUAL(glGetError(), (GLenum)GL_NO_ERROR);

	rendererOwner.reset();
	sourceOwner.reset();

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
}


/* resize() error guards (no GL frame needed), then a real effect: growing
 * the render position must grow the computed content position accordingly
 * (default fill_mode is FIT: the video's aspect ratio -- here 1:1, a square
 * frame -- is preserved, so a bigger square render area yields a bigger
 * square content area). */
static void testCxxVideoRendererResizeUpdatesContentPos()
{
	static constexpr uint32_t kWidth = 64;
	static constexpr uint32_t kHeight = 64;

	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	REQUIRE_GL_CONTEXT(glContext);

	struct pdraw_rect renderPos = {0, 0, kWidth, kHeight};
	struct pdraw_video_renderer_params rendererParams = {};
	VideoRendererTrackingListener rendererListener;
	IPdraw::IRawVideoSource *source = nullptr;
	IPdraw::IVideoRenderer *renderer = nullptr;
	setupRendererWithSource(loop,
				session,
				kWidth,
				kHeight,
				renderPos,
				rendererParams,
				mediaListener,
				rendererListener,
				&source,
				&renderer);
	auto rendererOwner = std::unique_ptr<IPdraw::IVideoRenderer>(renderer);
	auto sourceOwner = std::unique_ptr<IPdraw::IRawVideoSource>(source);

	pushFrameAndWaitRenderReady(
		loop, source, rendererListener, kWidth, kHeight, 0, 0);

	struct pdraw_rect contentPos = {};
	int ret = renderer->render(&contentPos);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_TRUE_FATAL(contentPos.width > 0);

	int badRet = renderer->resize(nullptr);
	CU_ASSERT_EQUAL(badRet, -EINVAL);
	struct pdraw_rect zeroRenderPos = {0, 0, 0, 0};
	badRet = renderer->resize(&zeroRenderPos);
	CU_ASSERT_EQUAL(badRet, -EPROTO);

	struct pdraw_rect biggerRenderPos = {0, 0, kWidth * 2, kHeight * 2};
	ret = renderer->resize(&biggerRenderPos);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	struct pdraw_rect contentPosAfterResize = {};
	ret = renderer->render(&contentPosAfterResize);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_TRUE(contentPosAfterResize.width > contentPos.width);
	CU_ASSERT_TRUE(contentPosAfterResize.height > contentPos.height);

	CU_ASSERT_EQUAL(glGetError(), (GLenum)GL_NO_ERROR);

	rendererOwner.reset();
	sourceOwner.reset();

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
}


/* setParams()/getParams() roundtrip -- no GL frame needed. Also documents a
 * subtle, deliberate quirk: enable_simplified_rendering is "only honored at
 * creation time" (see pdraw_defs.h), i.e. GlVideoRenderer::setParams()
 * preserves the value from creation regardless of what a later setParams()
 * call asks for, unlike every other field. */
static void testCxxVideoRendererSetParamsGetParamsRoundtrip()
{
	static constexpr uint32_t kWidth = 64;
	static constexpr uint32_t kHeight = 64;

	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	REQUIRE_GL_CONTEXT(glContext);

	struct pdraw_rect renderPos = {0, 0, kWidth, kHeight};
	struct pdraw_video_renderer_params rendererParams = {};
	rendererParams.enable_simplified_rendering = 0;
	VideoRendererTrackingListener rendererListener;
	IPdraw::IRawVideoSource *source = nullptr;
	IPdraw::IVideoRenderer *renderer = nullptr;
	setupRendererWithSource(loop,
				session,
				kWidth,
				kHeight,
				renderPos,
				rendererParams,
				mediaListener,
				rendererListener,
				&source,
				&renderer);
	auto rendererOwner = std::unique_ptr<IPdraw::IVideoRenderer>(renderer);
	auto sourceOwner = std::unique_ptr<IPdraw::IRawVideoSource>(source);

	struct pdraw_video_renderer_params initialParams = {};
	int ret = renderer->getParams(&initialParams);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_EQUAL(initialParams.fill_mode,
			PDRAW_VIDEO_RENDERER_FILL_MODE_FIT);
	CU_ASSERT_FALSE(initialParams.vertical_mirror);

	struct pdraw_video_renderer_params newParams = initialParams;
	newParams.fill_mode = PDRAW_VIDEO_RENDERER_FILL_MODE_CROP;
	newParams.vertical_mirror = true;
	newParams.enable_histograms = 1;
	newParams.enable_simplified_rendering = 1;
	ret = renderer->setParams(&newParams);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	struct pdraw_video_renderer_params updatedParams = {};
	ret = renderer->getParams(&updatedParams);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_EQUAL(updatedParams.fill_mode,
			PDRAW_VIDEO_RENDERER_FILL_MODE_CROP);
	CU_ASSERT_TRUE(updatedParams.vertical_mirror);
	CU_ASSERT_EQUAL(updatedParams.enable_histograms, 1);
	/* Unchanged: only honored at creation time. */
	CU_ASSERT_EQUAL(updatedParams.enable_simplified_rendering, 0);

	rendererOwner.reset();
	sourceOwner.reset();

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
}


/* Destroying the source removes its media: the renderer must report this
 * via onVideoRendererMediaRemoved(restart=false) (a plain teardown, not a
 * reconfiguration) and getMediaId() must reset to 0, matching
 * GlVideoRenderer::removeInputMedia(). A further render() call must not
 * crash, and (confirmed by reading render(), see the comment further down)
 * keeps showing the last successfully loaded frame rather than blanking. */
static void testCxxVideoRendererMediaRemovedWhenSourceDestroyed()
{
	static constexpr uint32_t kWidth = 64;
	static constexpr uint32_t kHeight = 64;

	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	REQUIRE_GL_CONTEXT(glContext);

	struct pdraw_rect renderPos = {0, 0, kWidth, kHeight};
	struct pdraw_video_renderer_params rendererParams = {};
	VideoRendererTrackingListener rendererListener;
	IPdraw::IRawVideoSource *source = nullptr;
	IPdraw::IVideoRenderer *renderer = nullptr;
	setupRendererWithSource(loop,
				session,
				kWidth,
				kHeight,
				renderPos,
				rendererParams,
				mediaListener,
				rendererListener,
				&source,
				&renderer);
	auto rendererOwner = std::unique_ptr<IPdraw::IVideoRenderer>(renderer);
	auto sourceOwner = std::unique_ptr<IPdraw::IRawVideoSource>(source);

	pushFrameAndWaitRenderReady(
		loop, source, rendererListener, kWidth, kHeight, 0, 0);

	struct pdraw_rect contentPos = {};
	int ret = renderer->render(&contentPos);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_TRUE_FATAL(contentPos.width > 0);
	CU_ASSERT_TRUE(renderer->getMediaId() != 0);

	sourceOwner.reset();
	source = nullptr;

	bool gotMediaRemoved = loop.pumpUntil(
		[&rendererListener]() {
			return rendererListener.mGotMediaRemoved;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotMediaRemoved);
	CU_ASSERT_FALSE(rendererListener.mLastRemovedRestart);
	CU_ASSERT_EQUAL(renderer->getMediaId(), 0u);

	/* GlVideoRenderer::render() keeps rendering mLoadedFrame (the last
	 * successfully loaded frame) once mFrameLoaded is true: that flag is
	 * never reset by removeInputMedia(), only mPrimaryMedia/mNextFrame
	 * are cleared -- confirmed by reading render()'s "actual rendering"
	 * section (renderVideoFrame() is called whenever mFrameLoaded, with
	 * no mPrimaryMedia check). A deliberate freeze-on-last-frame
	 * behavior, not a blank/black one: contentPos is therefore expected
	 * to stay exactly as before, not reset to zero. */
	struct pdraw_rect contentPosAfterRemoval = {};
	ret = renderer->render(&contentPosAfterRemoval);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(contentPosAfterRemoval.width, contentPos.width);
	CU_ASSERT_EQUAL(contentPosAfterRemoval.height, contentPos.height);

	CU_ASSERT_EQUAL(glGetError(), (GLenum)GL_NO_ERROR);

	rendererOwner.reset();

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
}

/* Verifies setMediaId() / getMediaId() by creating two raw video sources,
 * attaching the renderer to source1, then switching it to source2 mid-stream.
 *
 * Implementation details (confirmed by reading pdraw_renderer_video_gl.cpp):
 *   setMediaId(m2) sets mTargetPrimaryMediaId=m2 then schedules
 * idleRenewMedia() on the pomp loop.  When that idle fires: renewMedia() →
 * removeInputMedia(source1's media) → addMediaToVideoRenderer(m2, this) →
 * addInputMedia(source2's media) Both onVideoRendererMediaRemoved and
 * onVideoRendererMediaAdded fire synchronously within the same idle callback.
 *
 *   restart=false in onVideoRendererMediaRemoved because mPendingRestart is
 * only set by onChannelReconfigure / onChannelResolutionChange /
 * onChannelFramerateChange; setMediaId() does not trigger any of those.
 *
 *   After the switch, getMediaId() returns the new mPrimaryMediaId (=
 * media2Id), and frames pushed to source2's queue are routed through the newly
 * created mInputQueue to the renderer. */
static void testCxxVideoRendererSwitchMediaId()
{
	static constexpr uint32_t kWidth = 64;
	static constexpr uint32_t kHeight = 64;

	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	REQUIRE_GL_CONTEXT(glContext);

	struct pdraw_video_source_params sourceParams = {};
	sourceParams.playback_type = PDRAW_PLAYBACK_TYPE_LIVE;
	sourceParams.video.format = VDEF_FRAME_TYPE_RAW;
	sourceParams.video.raw.format = vdef_i420;
	sourceParams.video.raw.info.resolution.width = kWidth;
	sourceParams.video.raw.info.resolution.height = kHeight;
	sourceParams.video.raw.info.bit_depth = 8;
	sourceParams.video.raw.info.framerate.num = 30;
	sourceParams.video.raw.info.framerate.den = 1;

	/* --- Source 1 and its media --- */
	IPdraw::IRawVideoSource *source1 = nullptr;
	int ret = session->createRawVideoSource(
		&sourceParams, &g_stub_raw_video_source_listener, &source1);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source1);
	auto source1Owner = std::unique_ptr<IPdraw::IRawVideoSource>(source1);

	bool gotMedia1 = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mAdded.size() >= 1; },
		15000);
	CU_ASSERT_TRUE_FATAL(gotMedia1);
	unsigned int media1Id = mediaListener.mAdded[0].id;

	/* --- Source 2 and its media --- */
	IPdraw::IRawVideoSource *source2 = nullptr;
	ret = session->createRawVideoSource(
		&sourceParams, &g_stub_raw_video_source_listener, &source2);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(source2);
	auto source2Owner = std::unique_ptr<IPdraw::IRawVideoSource>(source2);

	bool gotMedia2 = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mAdded.size() >= 2; },
		15000);
	CU_ASSERT_TRUE_FATAL(gotMedia2);
	unsigned int media2Id = mediaListener.mAdded[1].id;
	CU_ASSERT_NOT_EQUAL(media1Id, media2Id);

	/* --- Renderer on source1's media --- */
	struct pdraw_rect renderPos = {0, 0, kWidth, kHeight};
	struct pdraw_video_renderer_params rendererParams = {};
	VideoRendererTrackingListener rendererListener;
	IPdraw::IVideoRenderer *renderer = nullptr;
	ret = session->createVideoRenderer(media1Id,
					   &renderPos,
					   &rendererParams,
					   &rendererListener,
					   &renderer);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(renderer);
	auto rendererOwner = std::unique_ptr<IPdraw::IVideoRenderer>(renderer);

	bool gotRendererMedia1 = loop.pumpUntil(
		[&rendererListener]() {
			return rendererListener.mGotMediaAdded;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotRendererMedia1);
	CU_ASSERT_EQUAL(renderer->getMediaId(), media1Id);

	/* --- Push one frame from source1 and render it --- */
	pushFrameAndWaitRenderReady(
		loop, source1, rendererListener, kWidth, kHeight, 0, 0);
	struct pdraw_rect contentPos1 = {};
	ret = renderer->render(&contentPos1);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_TRUE_FATAL(contentPos1.width > 0);

	/* --- Switch renderer to source2 --- */
	rendererListener.mGotMediaAdded = false;
	rendererListener.mGotMediaRemoved = false;

	ret = renderer->setMediaId(media2Id);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	/* renewMedia() fires as an idle callback: removeInputMedia (fires
	 * onVideoRendererMediaRemoved) then addMediaToVideoRenderer (fires
	 * onVideoRendererMediaAdded) happen in the same iteration. */
	bool gotRemoved = loop.pumpUntil(
		[&rendererListener]() {
			return rendererListener.mGotMediaRemoved;
		},
		15000);
	CU_ASSERT_TRUE(gotRemoved);
	/* setMediaId() does not trigger a channel reconfigure/resolutionChange,
	 * so mPendingRestart was false → restart=false in the callback. */
	CU_ASSERT_FALSE(rendererListener.mLastRemovedRestart);

	bool gotAdded2 = loop.pumpUntil(
		[&rendererListener]() {
			return rendererListener.mGotMediaAdded;
		},
		15000);
	CU_ASSERT_TRUE(gotAdded2);
	CU_ASSERT_EQUAL(renderer->getMediaId(), media2Id);

	/* --- Push one frame from source2 and render it --- */
	pushFrameAndWaitRenderReady(
		loop, source2, rendererListener, kWidth, kHeight, 0, 1);
	struct pdraw_rect contentPos2 = {};
	ret = renderer->render(&contentPos2);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_TRUE(contentPos2.width > 0);

	CU_ASSERT_EQUAL(glGetError(), (GLenum)GL_NO_ERROR);

	rendererOwner.reset();
	source1Owner.reset();
	source2Owner.reset();

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
}


/* GlVideo::renderBlur() is only reached through GlVideoRenderer::render()'s
 * "if (mFrameLoaded) ... renderVideoFrame()" branch (pdraw_renderer_video_gl.
 * cpp:2486-2506); when no frame has ever been loaded yet, render() instead
 * takes the "else" branch and just calls GlVideo::clear() -- no
 * GlVideo::renderFrame() call at all, so renderBlur()/renderPadding() can
 * never run regardless of any transition state.
 *
 * Critically, GlVideoRenderer::doTransition() (called from render(), pdraw_
 * renderer_video_gl.cpp:1319) forces its output "loadFrame" to false while a
 * FADE_TO_BLUR transition is within its hold window, *unless* mFrameLoaded
 * was already true -- FADE_TO_BLUR is not one of the transitions exempted at
 * pdraw_renderer_video_gl.cpp:1400-1404 (only NONE/FADE_FROM_BLACK/
 * FLASH_THEN_BLACK_AND_WHITE are). So triggering RECONFIGURE (which arms
 * FADE_TO_BLUR, pdraw_renderer_video_gl.cpp:877-879) before the first frame
 * is ever loaded leaves mFrameLoaded permanently false: every subsequent
 * render() call keeps taking the clear()-only branch, and renderBlur() is
 * never invoked -- confirmed empirically (not just by reading the source):
 * an earlier version of this test that skipped the pre-transition render()
 * call never logged "frame #N loaded"/"render state: video" at all, for any
 * of its render() calls.
 *
 * Fix: render() once right after pushing the first frame, before sending any
 * downstream event, so mFrameLoaded is already true when RECONFIGURE arrives.
 * Once true, mFrameLoaded is never reset by any of the events exercised below
 * (confirmed by reading onChannel{Reconfigure,Eos,...}: none of them touch
 * mFrameLoaded), so every later render() call keeps rendering the same
 * loaded frame -- exactly what a transition/blur overlay is supposed to do. */
static void testCxxVideoRendererBlurAndTransitions()
{
	static constexpr uint32_t kWidth = 64;
	static constexpr uint32_t kHeight = 64;

	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	REQUIRE_GL_CONTEXT(glContext);

	struct pdraw_rect renderPos = {0, 0, kWidth, kHeight};
	struct pdraw_video_renderer_params rendererParams = {};
	rendererParams.enable_transition_flags =
		PDRAW_VIDEO_RENDERER_TRANSITION_FLAG_ALL;
	VideoRendererTrackingListener rendererListener;
	IPdraw::IRawVideoSource *source = nullptr;
	IPdraw::IVideoRenderer *renderer = nullptr;

	setupRendererWithSource(loop,
				session,
				kWidth,
				kHeight,
				renderPos,
				rendererParams,
				mediaListener,
				rendererListener,
				&source,
				&renderer);
	auto rendererOwner = std::unique_ptr<IPdraw::IVideoRenderer>(renderer);
	auto sourceOwner = std::unique_ptr<IPdraw::IRawVideoSource>(source);

	pushFrameAndWaitRenderReady(
		loop, source, rendererListener, kWidth, kHeight, 0, 0);

	VideoRendererWrapper *wrapper =
		static_cast<VideoRendererWrapper *>(renderer);
	GlVideoRenderer *glRenderer =
		static_cast<GlVideoRenderer *>(wrapper->getElement());
	CU_ASSERT_PTR_NOT_NULL_FATAL(glRenderer);

	unsigned int inputCount = glRenderer->getInputMediaCount();
	CU_ASSERT_EQUAL(inputCount, 1);
	Media *media = glRenderer->getInputMedia(0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(media);
	Channel *channel = glRenderer->getInputChannel(media);
	CU_ASSERT_PTR_NOT_NULL_FATAL(channel);

	/* 0. Load and render the pushed frame *before* any transition is
	 * armed: mFrameLoaded must already be true when RECONFIGURE arrives,
	 * see the comment above this test. */
	struct pdraw_rect contentPos = {};
	int ret = renderer->render(&contentPos);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_TRUE_FATAL(contentPos.width > 0);

	/* 1. Trigger RECONFIGURE downstream event -> Transition::FADE_TO_BLUR
	 */
	int err = channel->sendDownstreamEvent(
		Channel::DownstreamEvent::RECONFIGURE);
	CU_ASSERT_EQUAL(err, 0);

	memset(&contentPos, 0, sizeof(contentPos));
	ret = renderer->render(&contentPos);
	CU_ASSERT_EQUAL(ret, 0);
	/* mFrameLoaded was already true (step 0): render() takes the
	 * renderVideoFrame()/GlVideo::renderFrame() branch (not the
	 * clear()-only one), which is what actually calls renderBlur() now
	 * that mApplyBlur is set by updateTransition(). contentPos must
	 * therefore still reflect the last loaded frame, not the zeroed-out
	 * value clear()'s branch would have left it at. */
	CU_ASSERT_TRUE(contentPos.width > 0);

	/* 2. Trigger other transitions to cover doTransition/updateTransition
	 */
	err = channel->sendDownstreamEvent(Channel::DownstreamEvent::SOS);
	CU_ASSERT_EQUAL(err, 0);
	ret = renderer->render(&contentPos);
	CU_ASSERT_EQUAL(ret, 0);

	err = channel->sendDownstreamEvent(Channel::DownstreamEvent::EOS);
	CU_ASSERT_EQUAL(err, 0);
	ret = renderer->render(&contentPos);
	CU_ASSERT_EQUAL(ret, 0);

	err = channel->sendDownstreamEvent(Channel::DownstreamEvent::TIMEOUT);
	CU_ASSERT_EQUAL(err, 0);
	ret = renderer->render(&contentPos);
	CU_ASSERT_EQUAL(ret, 0);

	err = channel->sendDownstreamEvent(
		Channel::DownstreamEvent::PHOTO_TRIGGER);
	CU_ASSERT_EQUAL(err, 0);
	ret = renderer->render(&contentPos);
	CU_ASSERT_EQUAL(ret, 0);

	/* 3. Trigger abortTransition by starting a new transition before the
	 * old one finishes */
	err = channel->sendDownstreamEvent(
		Channel::DownstreamEvent::RECONFIGURE);
	CU_ASSERT_EQUAL(err, 0);
	err = channel->sendDownstreamEvent(Channel::DownstreamEvent::SOS);
	CU_ASSERT_EQUAL(err, 0);
	ret = renderer->render(&contentPos);
	CU_ASSERT_EQUAL(ret, 0);

	CU_ASSERT_EQUAL(glGetError(), (GLenum)GL_NO_ERROR);

	rendererOwner.reset();
	sourceOwner.reset();

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
}


/* Exercises GlVideo::renderPadding()'s real drawing path (the 4-pass
 * downscale/blur/render of the padded background), not just its early
 * no-op returns for FIT/CROP fill modes (pdraw_gl_video.cpp:1960-1972).
 *
 * Two conditions are required to actually reach that drawing code, both
 * confirmed by reading pdraw_gl_video.cpp before writing this test:
 *   1. fill_mode = FIT_PAD_BLUR_EXTEND (or _CROP): setupPaddingFbo() only
 *      allocates mPaddingFbo[]/mPaddingFboTexture[] for these two modes
 *      (early-return guard at pdraw_gl_video.cpp:1822-1824); without them
 *      renderPadding() would have nothing to render into even if called.
 *   2. renderPos's aspect ratio must differ from the video's: renderFrame()
 *      only calls renderPadding() when videoAR != windowAR
 *      (pdraw_gl_video.cpp:~3444) -- no padding is ever needed once the
 *      video already fills renderPos exactly.
 * The video is a 64x64 square (AR=1) rendered into a 128x64 (AR=2) target,
 * guaranteeing left/right padding and satisfying both conditions. */
static void testCxxVideoRendererFillModePadBlurExtend()
{
	static constexpr uint32_t kWidth = 64;
	static constexpr uint32_t kHeight = 64;
	static constexpr uint32_t kRenderWidth = 128;
	static constexpr uint32_t kRenderHeight = 64;

	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	REQUIRE_GL_CONTEXT(glContext);

	struct pdraw_rect renderPos = {0, 0, kRenderWidth, kRenderHeight};
	struct pdraw_video_renderer_params rendererParams = {};
	rendererParams.fill_mode =
		PDRAW_VIDEO_RENDERER_FILL_MODE_FIT_PAD_BLUR_EXTEND;
	VideoRendererTrackingListener rendererListener;
	IPdraw::IRawVideoSource *source = nullptr;
	IPdraw::IVideoRenderer *renderer = nullptr;
	setupRendererWithSource(loop,
				session,
				kWidth,
				kHeight,
				renderPos,
				rendererParams,
				mediaListener,
				rendererListener,
				&source,
				&renderer);
	auto rendererOwner = std::unique_ptr<IPdraw::IVideoRenderer>(renderer);
	auto sourceOwner = std::unique_ptr<IPdraw::IRawVideoSource>(source);

	pushFrameAndWaitRenderReady(
		loop, source, rendererListener, kWidth, kHeight, 0, 0);

	struct pdraw_rect contentPos = {};
	int ret = renderer->render(&contentPos);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_TRUE(contentPos.width > 0);
	CU_ASSERT_TRUE(contentPos.height > 0);
	/* Square video letterboxed into a wider target keeps a 1:1 AR and is
	 * therefore narrower than the render target -- confirms padding is
	 * actually needed on the sides (i.e. this test's premise holds). */
	CU_ASSERT_TRUE(contentPos.width < kRenderWidth);

	/* Second render() call: exercises the steady-state path (padding
	 * FBOs already allocated by the first call) in addition to the
	 * first-render setup path. */
	ret = renderer->render(&contentPos);
	CU_ASSERT_EQUAL(ret, 0);

	CU_ASSERT_EQUAL(glGetError(), (GLenum)GL_NO_ERROR);

	rendererOwner.reset();
	sourceOwner.reset();

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
}


/* Same setup as testCxxVideoRendererFillModePadBlurExtend, but with
 * vertical_mirror=true, to reach renderPadding()'s other tex-coord branch
 * (the "else" at pdraw_gl_video.cpp:2136).
 *
 * Confirmed by reading renderPadding() before writing this test: it starts
 * with "bool mirrorTexture = verticalMirror;" (pdraw_gl_video.cpp:1954), but
 * every YUV/GRAY program branch -- which covers every raw format this suite
 * uses, including I420 -- immediately negates it right after
 * ("mirrorTexture = !verticalMirror;", e.g. pdraw_gl_video.cpp:2006 for
 * YUV_TO_RGB_PLANAR). So testCxxVideoRendererFillModePadBlurExtend, which
 * leaves vertical_mirror at its default false, actually already runs with
 * mirrorTexture=true; only setting vertical_mirror=true here flips
 * mirrorTexture to false and hits the branch neither test previously
 * covered. (NOCONV -- the ext-texture passthrough program -- is the only
 * program that does not negate, but this suite never renders through it in
 * FIT_PAD_BLUR_EXTEND mode.) */
static void testCxxVideoRendererFillModePadBlurExtendMirrored()
{
	static constexpr uint32_t kWidth = 64;
	static constexpr uint32_t kHeight = 64;
	static constexpr uint32_t kRenderWidth = 128;
	static constexpr uint32_t kRenderHeight = 64;

	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	REQUIRE_GL_CONTEXT(glContext);

	struct pdraw_rect renderPos = {0, 0, kRenderWidth, kRenderHeight};
	struct pdraw_video_renderer_params rendererParams = {};
	rendererParams.fill_mode =
		PDRAW_VIDEO_RENDERER_FILL_MODE_FIT_PAD_BLUR_EXTEND;
	rendererParams.vertical_mirror = true;
	VideoRendererTrackingListener rendererListener;
	IPdraw::IRawVideoSource *source = nullptr;
	IPdraw::IVideoRenderer *renderer = nullptr;
	setupRendererWithSource(loop,
				session,
				kWidth,
				kHeight,
				renderPos,
				rendererParams,
				mediaListener,
				rendererListener,
				&source,
				&renderer);
	auto rendererOwner = std::unique_ptr<IPdraw::IVideoRenderer>(renderer);
	auto sourceOwner = std::unique_ptr<IPdraw::IRawVideoSource>(source);

	pushFrameAndWaitRenderReady(
		loop, source, rendererListener, kWidth, kHeight, 0, 0);

	struct pdraw_rect contentPos = {};
	int ret = renderer->render(&contentPos);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_TRUE(contentPos.width > 0);
	CU_ASSERT_TRUE(contentPos.width < kRenderWidth);

	/* Second render() call, same reasoning as the non-mirrored test:
	 * exercises the steady-state path once the padding FBOs already
	 * exist. */
	ret = renderer->render(&contentPos);
	CU_ASSERT_EQUAL(ret, 0);

	CU_ASSERT_EQUAL(glGetError(), (GLenum)GL_NO_ERROR);

	rendererOwner.reset();
	sourceOwner.reset();

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
}


/* Exercises GlVideo::updateZebra()'s "enabled" branch (pdraw_gl_video.cpp:
 * 2358-2391), called from the main (non-padding, non-blur) render path as
 * "updateZebra(contentPos, prog, params->enable_overexposure_zebras,
 * params->overexposure_zebras_threshold)" (pdraw_gl_video.cpp:3585-3588).
 *
 * Every other test in this file leaves enable_overexposure_zebras at its
 * zero-initialized default, so the only calls to updateZebra() they ever
 * reach are the "disable" ones made with a null contentPos from
 * renderBlur()/renderPadding() (to force zebras off during those passes,
 * e.g. pdraw_gl_video.cpp:1614/2081). With enable=true and a real
 * contentPos, updateZebra() additionally computes and uploads a
 * time-based zebra_phase and a contentPos-width-scaled zebra_weight
 * uniform -- neither previously covered by this suite. */
static void testCxxVideoRendererOverexposureZebrasEnabled()
{
	static constexpr uint32_t kWidth = 64;
	static constexpr uint32_t kHeight = 64;

	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	REQUIRE_GL_CONTEXT(glContext);

	struct pdraw_rect renderPos = {0, 0, kWidth, kHeight};
	struct pdraw_video_renderer_params rendererParams = {};
	rendererParams.enable_overexposure_zebras = 1;
	rendererParams.overexposure_zebras_threshold = 0.7f;
	VideoRendererTrackingListener rendererListener;
	IPdraw::IRawVideoSource *source = nullptr;
	IPdraw::IVideoRenderer *renderer = nullptr;
	setupRendererWithSource(loop,
				session,
				kWidth,
				kHeight,
				renderPos,
				rendererParams,
				mediaListener,
				rendererListener,
				&source,
				&renderer);
	auto rendererOwner = std::unique_ptr<IPdraw::IVideoRenderer>(renderer);
	auto sourceOwner = std::unique_ptr<IPdraw::IRawVideoSource>(source);

	pushFrameAndWaitRenderReady(
		loop, source, rendererListener, kWidth, kHeight, 0, 0);

	struct pdraw_rect contentPos = {};
	int ret = renderer->render(&contentPos);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_TRUE(contentPos.width > 0);

	/* Second render(): zebra_phase is derived from time_get_monotonic(),
	 * so a second call exercises the same uniform-upload path with a
	 * different phase value. */
	ret = renderer->render(&contentPos);
	CU_ASSERT_EQUAL(ret, 0);

	CU_ASSERT_EQUAL(glGetError(), (GLenum)GL_NO_ERROR);

	rendererOwner.reset();
	sourceOwner.reset();

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
}


/* Exercises Program::NOCONV in renderPadding(), computeHistograms() and
 * renderBlur() -- none of this file's other tests reaches that program in
 * any of the three, since none combine the ext-texture path with padding/
 * histograms/a transition.
 *
 * Confirmed by reading the source before writing this test: whichever
 * Program a render() call uses is decided by GlVideo::getProgram() from the
 * *format* argument passed into GlVideo::renderFrame() -- and for the
 * ext-texture path specifically, GlVideoRenderer::renderExternalVideoFrame()
 * always passes &vdef_rgb (pdraw_renderer_video_gl.cpp:1690), which
 * getProgram() maps to Program::NOCONV (pdraw_gl_video.cpp:1122-1123),
 * regardless of the raw video source's own declared format. So the ext-
 * texture path is the only way to reach NOCONV in these three functions at
 * all (they are never called with a source format of vdef_rgb/vdef_opaque
 * elsewhere in this suite, and GlVideo::loadFrame()'s own NOCONV case is a
 * no-op -- pdraw_gl_video.cpp:3082-3084 -- so a non-ext-texture RGB source
 * would not even upload pixel data).
 *
 * All three are combined in a single render() call by reusing the DAR setup
 * from testCxxVideoRendererExtTextureWithDAR (64x64 render target vs. a 2:1
 * DAR video -- an AR mismatch, to force renderPadding()) together with
 * enable_histograms (for computeHistograms()) and a RECONFIGURE-triggered
 * transition (for renderBlur()) -- GlVideo::renderFrame() calls
 * computeHistograms() unconditionally, then renderPadding() when
 * videoAR != windowAR, then renderBlur() when mApplyBlur is set
 * (pdraw_gl_video.cpp:3364-3469), so nothing here needs three separate
 * render() calls. As in testCxxVideoRendererBlurAndTransitions, the frame
 * must be loaded (mFrameLoaded=true, via one primed render() call) *before*
 * RECONFIGURE is sent, or the transition's frame-load suppression leaves
 * mFrameLoaded permanently false and none of the three ever runs -- see the
 * comment on that test for the full mechanism. */
static void testCxxVideoRendererExtTexturePadHistogramBlur()
{
	static constexpr uint32_t kWidth = 64;
	static constexpr uint32_t kHeight = 64;

	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	REQUIRE_GL_CONTEXT(glContext);

	struct pdraw_rect renderPos = {0, 0, kWidth, kHeight};
	struct pdraw_video_renderer_params rendererParams = {};
	rendererParams.fill_mode =
		PDRAW_VIDEO_RENDERER_FILL_MODE_FIT_PAD_BLUR_EXTEND;
	rendererParams.enable_histograms = 1;
	rendererParams.enable_transition_flags =
		PDRAW_VIDEO_RENDERER_TRANSITION_FLAG_ALL;
	rendererParams.video_texture_width = 64;
	rendererParams.video_texture_dar_width = 2;
	rendererParams.video_texture_dar_height = 1;

	ExtTexHistogramRendererListener rendererListener;
	IPdraw::IRawVideoSource *source = nullptr;
	IPdraw::IVideoRenderer *renderer = nullptr;
	setupRendererWithSource(loop,
				session,
				kWidth,
				kHeight,
				renderPos,
				rendererParams,
				mediaListener,
				rendererListener,
				&source,
				&renderer);
	auto rendererOwner = std::unique_ptr<IPdraw::IVideoRenderer>(renderer);
	auto sourceOwner = std::unique_ptr<IPdraw::IRawVideoSource>(source);

	pushFrameAndWaitRenderReady(
		loop, source, rendererListener, kWidth, kHeight, 0, 0);

	/* Prime mFrameLoaded=true before arming any transition. */
	struct pdraw_rect contentPos = {};
	int ret = renderer->render(&contentPos);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_TRUE_FATAL(contentPos.width > 0);
	CU_ASSERT_TRUE(rendererListener.mLoadCallCount >= 1);

	VideoRendererWrapper *wrapper =
		static_cast<VideoRendererWrapper *>(renderer);
	GlVideoRenderer *glRenderer =
		static_cast<GlVideoRenderer *>(wrapper->getElement());
	CU_ASSERT_PTR_NOT_NULL_FATAL(glRenderer);
	Media *media = glRenderer->getInputMedia(0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(media);
	Channel *channel = glRenderer->getInputChannel(media);
	CU_ASSERT_PTR_NOT_NULL_FATAL(channel);

	int err = channel->sendDownstreamEvent(
		Channel::DownstreamEvent::RECONFIGURE);
	CU_ASSERT_EQUAL(err, 0);

	/* This single render() call exercises renderPadding(),
	 * computeHistograms() and renderBlur() together, all with
	 * Program::NOCONV. */
	memset(&contentPos, 0, sizeof(contentPos));
	ret = renderer->render(&contentPos);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_TRUE(contentPos.width > 0);
	CU_ASSERT_TRUE(rendererListener.mOverlayCallCount >= 1);
	CU_ASSERT_EQUAL(
		rendererListener.mHistogramLen[PDRAW_HISTOGRAM_CHANNEL_LUMA],
		256u);

	CU_ASSERT_EQUAL(glGetError(), (GLenum)GL_NO_ERROR);

	rendererOwner.reset();
	sourceOwner.reset();

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
}


/* Exercises Program::YUV_TO_RGB_SEMIPLANAR in renderPadding(),
 * computeHistograms() and renderBlur() using an NV21 (2-plane, swapped U/V)
 * source. Every other format-specific test in this file that reaches
 * SEMIPLANAR uses NV12 instead (swapUv=false); GlVideo::getProgram() maps
 * vdef_nv21 to YUV_TO_RGB_SEMIPLANAR with swapUv=true (pdraw_gl_video.cpp:
 * 1109-1111) -- the only supported raw format that sets swapUv at all. So
 * this test doubles as the only coverage of fillYuv2RgbMatrix()'s
 * swapUv=true branch (pdraw_gl_video.cpp:1144-1156), which every one of
 * renderPadding()/renderBlur()/computeHistograms()/the main per-pixel render
 * path calls with the same locally-computed swapUv.
 *
 * Same combined-render() approach as
 * testCxxVideoRendererExtTexturePadHistogramBlur (see that test's comment
 * for why one render() call reaches all three functions, and why a priming
 * render() call must happen before RECONFIGURE): FIT_PAD_BLUR_EXTEND with an
 * AR-mismatched render target for renderPadding(), enable_histograms for
 * computeHistograms(), and a RECONFIGURE-triggered transition for
 * renderBlur(). */
static void testCxxVideoRendererNv21PadHistogramBlur()
{
	static constexpr uint32_t kWidth = 64;
	static constexpr uint32_t kHeight = 64;
	static constexpr uint32_t kRenderWidth = 128;
	static constexpr uint32_t kRenderHeight = 64;

	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	REQUIRE_GL_CONTEXT(glContext);

	struct pdraw_rect renderPos = {0, 0, kRenderWidth, kRenderHeight};
	struct pdraw_video_renderer_params rendererParams = {};
	rendererParams.fill_mode =
		PDRAW_VIDEO_RENDERER_FILL_MODE_FIT_PAD_BLUR_EXTEND;
	rendererParams.enable_histograms = 1;
	rendererParams.enable_transition_flags =
		PDRAW_VIDEO_RENDERER_TRANSITION_FLAG_ALL;

	HistogramCapturingRendererListener rendererListener;
	IPdraw::IRawVideoSource *source = nullptr;
	IPdraw::IVideoRenderer *renderer = nullptr;
	setupRendererWithSource(loop,
				session,
				kWidth,
				kHeight,
				renderPos,
				rendererParams,
				mediaListener,
				rendererListener,
				&source,
				&renderer,
				vdef_nv21,
				8);
	auto rendererOwner = std::unique_ptr<IPdraw::IVideoRenderer>(renderer);
	auto sourceOwner = std::unique_ptr<IPdraw::IRawVideoSource>(source);

	pushFrameAndWaitRenderReady(loop,
				    source,
				    rendererListener,
				    kWidth,
				    kHeight,
				    0,
				    0,
				    vdef_nv21,
				    8);

	/* Prime mFrameLoaded=true before arming any transition. */
	struct pdraw_rect contentPos = {};
	int ret = renderer->render(&contentPos);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_TRUE_FATAL(contentPos.width > 0);
	CU_ASSERT_TRUE(contentPos.width < kRenderWidth);

	VideoRendererWrapper *wrapper =
		static_cast<VideoRendererWrapper *>(renderer);
	GlVideoRenderer *glRenderer =
		static_cast<GlVideoRenderer *>(wrapper->getElement());
	CU_ASSERT_PTR_NOT_NULL_FATAL(glRenderer);
	Media *media = glRenderer->getInputMedia(0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(media);
	Channel *channel = glRenderer->getInputChannel(media);
	CU_ASSERT_PTR_NOT_NULL_FATAL(channel);

	int err = channel->sendDownstreamEvent(
		Channel::DownstreamEvent::RECONFIGURE);
	CU_ASSERT_EQUAL(err, 0);

	/* This single render() call exercises renderPadding(),
	 * computeHistograms() and renderBlur() together, all with
	 * Program::YUV_TO_RGB_SEMIPLANAR / swapUv=true. */
	memset(&contentPos, 0, sizeof(contentPos));
	ret = renderer->render(&contentPos);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_TRUE(contentPos.width > 0);
	CU_ASSERT_TRUE(rendererListener.mOverlayCallCount >= 1);
	CU_ASSERT_EQUAL(
		rendererListener.mHistogramLen[PDRAW_HISTOGRAM_CHANNEL_LUMA],
		256u);

	CU_ASSERT_EQUAL(glGetError(), (GLenum)GL_NO_ERROR);

	rendererOwner.reset();
	sourceOwner.reset();

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
}


/* Exercises GlVideoRenderer::onChannelResolutionChange() (pdraw_renderer_
 * video_gl.cpp:886-906), never reached by any other test in this file
 * (testCxxVideoRendererBlurAndTransitions sends RECONFIGURE/SOS/EOS/TIMEOUT/
 * PHOTO_TRIGGER, but never RESOLUTION_CHANGE).
 *
 * onChannelResolutionChange() unconditionally sets mPendingRestart=true, then
 * -- only if mCurrentTransition==NONE, "low priority transition, do not
 * trigger transition if another transition is ongoing" -- arms
 * Transition::FADE_TO_BLACK_AND_WHITE when TRANSITION_FLAG_TIMEOUT is
 * enabled (the same flag TIMEOUT itself uses, not a dedicated
 * RESOLUTION_CHANGE flag). This test sends RESOLUTION_CHANGE as the very
 * first event so mCurrentTransition is still NONE, covering both the
 * unconditional and the conditional line.
 *
 * mPendingRestart has no public getter either, but (as already documented by
 * testCxxVideoRendererMediaRemovedWhenSourceDestroyed, which asserts
 * restart=false specifically *because* no reconfigure/resolution/framerate
 * event was ever sent there) it surfaces as the "restart" argument of
 * onVideoRendererMediaRemoved() once the source is destroyed -- reused here
 * as the observable proof that this exact function set it to true. */
static void testCxxVideoRendererResolutionChangeSetsRestartFlag()
{
	static constexpr uint32_t kWidth = 64;
	static constexpr uint32_t kHeight = 64;

	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	REQUIRE_GL_CONTEXT(glContext);

	struct pdraw_rect renderPos = {0, 0, kWidth, kHeight};
	struct pdraw_video_renderer_params rendererParams = {};
	rendererParams.enable_transition_flags =
		PDRAW_VIDEO_RENDERER_TRANSITION_FLAG_ALL;
	VideoRendererTrackingListener rendererListener;
	IPdraw::IRawVideoSource *source = nullptr;
	IPdraw::IVideoRenderer *renderer = nullptr;
	setupRendererWithSource(loop,
				session,
				kWidth,
				kHeight,
				renderPos,
				rendererParams,
				mediaListener,
				rendererListener,
				&source,
				&renderer);
	auto rendererOwner = std::unique_ptr<IPdraw::IVideoRenderer>(renderer);
	auto sourceOwner = std::unique_ptr<IPdraw::IRawVideoSource>(source);

	pushFrameAndWaitRenderReady(
		loop, source, rendererListener, kWidth, kHeight, 0, 0);

	/* Prime mFrameLoaded=true before arming any transition (same reason
	 * as testCxxVideoRendererBlurAndTransitions). */
	struct pdraw_rect contentPos = {};
	int ret = renderer->render(&contentPos);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_TRUE_FATAL(contentPos.width > 0);

	VideoRendererWrapper *wrapper =
		static_cast<VideoRendererWrapper *>(renderer);
	GlVideoRenderer *glRenderer =
		static_cast<GlVideoRenderer *>(wrapper->getElement());
	CU_ASSERT_PTR_NOT_NULL_FATAL(glRenderer);
	Media *media = glRenderer->getInputMedia(0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(media);
	Channel *channel = glRenderer->getInputChannel(media);
	CU_ASSERT_PTR_NOT_NULL_FATAL(channel);

	int err = channel->sendDownstreamEvent(
		Channel::DownstreamEvent::RESOLUTION_CHANGE);
	CU_ASSERT_EQUAL(err, 0);

	ret = renderer->render(&contentPos);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_TRUE(contentPos.width > 0);

	sourceOwner.reset();
	source = nullptr;

	bool gotMediaRemoved = loop.pumpUntil(
		[&rendererListener]() {
			return rendererListener.mGotMediaRemoved;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotMediaRemoved);
	CU_ASSERT_TRUE(rendererListener.mLastRemovedRestart);

	CU_ASSERT_EQUAL(glGetError(), (GLenum)GL_NO_ERROR);

	rendererOwner.reset();

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
}


/* Same as testCxxVideoRendererResolutionChangeSetsRestartFlag but for
 * GlVideoRenderer::onChannelFramerateChange() (pdraw_renderer_video_gl.cpp:
 * 910-930) -- structurally identical (mPendingRestart unconditional,
 * FADE_TO_BLACK_AND_WHITE conditional on mCurrentTransition==NONE), but a
 * distinct function/downstream event, so it needs its own coverage. */
static void testCxxVideoRendererFramerateChangeSetsRestartFlag()
{
	static constexpr uint32_t kWidth = 64;
	static constexpr uint32_t kHeight = 64;

	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	REQUIRE_GL_CONTEXT(glContext);

	struct pdraw_rect renderPos = {0, 0, kWidth, kHeight};
	struct pdraw_video_renderer_params rendererParams = {};
	rendererParams.enable_transition_flags =
		PDRAW_VIDEO_RENDERER_TRANSITION_FLAG_ALL;
	VideoRendererTrackingListener rendererListener;
	IPdraw::IRawVideoSource *source = nullptr;
	IPdraw::IVideoRenderer *renderer = nullptr;
	setupRendererWithSource(loop,
				session,
				kWidth,
				kHeight,
				renderPos,
				rendererParams,
				mediaListener,
				rendererListener,
				&source,
				&renderer);
	auto rendererOwner = std::unique_ptr<IPdraw::IVideoRenderer>(renderer);
	auto sourceOwner = std::unique_ptr<IPdraw::IRawVideoSource>(source);

	pushFrameAndWaitRenderReady(
		loop, source, rendererListener, kWidth, kHeight, 0, 0);

	struct pdraw_rect contentPos = {};
	int ret = renderer->render(&contentPos);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_TRUE_FATAL(contentPos.width > 0);

	VideoRendererWrapper *wrapper =
		static_cast<VideoRendererWrapper *>(renderer);
	GlVideoRenderer *glRenderer =
		static_cast<GlVideoRenderer *>(wrapper->getElement());
	CU_ASSERT_PTR_NOT_NULL_FATAL(glRenderer);
	Media *media = glRenderer->getInputMedia(0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(media);
	Channel *channel = glRenderer->getInputChannel(media);
	CU_ASSERT_PTR_NOT_NULL_FATAL(channel);

	int err = channel->sendDownstreamEvent(
		Channel::DownstreamEvent::FRAMERATE_CHANGE);
	CU_ASSERT_EQUAL(err, 0);

	ret = renderer->render(&contentPos);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_TRUE(contentPos.width > 0);

	sourceOwner.reset();
	source = nullptr;

	bool gotMediaRemoved = loop.pumpUntil(
		[&rendererListener]() {
			return rendererListener.mGotMediaRemoved;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotMediaRemoved);
	CU_ASSERT_TRUE(rendererListener.mLastRemovedRestart);

	CU_ASSERT_EQUAL(glGetError(), (GLenum)GL_NO_ERROR);

	rendererOwner.reset();

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
}


/* Exercises GlVideoRenderer::onChannelSessionMetaUpdate() (pdraw_renderer_
 * video_gl.cpp:2149-2182), reached via IRawVideoSource::setSessionMetadata()
 * (pdraw_external_raw_video_source.cpp:423-442), which updates the source's
 * output media sessionMeta and sends Channel::DownstreamEvent::
 * SESSION_META_UPDATE downstream -- not exercised by any other test in this
 * file. SessionMetaCapturingRendererListener observes the effect through the
 * renderVideoOverlay() mediaInfo argument (see that class's comment). */
static void testCxxVideoRendererSessionMetaUpdate()
{
	static constexpr uint32_t kWidth = 64;
	static constexpr uint32_t kHeight = 64;

	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	REQUIRE_GL_CONTEXT(glContext);

	struct pdraw_rect renderPos = {0, 0, kWidth, kHeight};
	struct pdraw_video_renderer_params rendererParams = {};
	SessionMetaCapturingRendererListener rendererListener;
	IPdraw::IRawVideoSource *source = nullptr;
	IPdraw::IVideoRenderer *renderer = nullptr;
	setupRendererWithSource(loop,
				session,
				kWidth,
				kHeight,
				renderPos,
				rendererParams,
				mediaListener,
				rendererListener,
				&source,
				&renderer);
	auto rendererOwner = std::unique_ptr<IPdraw::IVideoRenderer>(renderer);
	auto sourceOwner = std::unique_ptr<IPdraw::IRawVideoSource>(source);

	pushFrameAndWaitRenderReady(
		loop, source, rendererListener, kWidth, kHeight, 0, 0);

	struct pdraw_rect contentPos = {};
	int ret = renderer->render(&contentPos);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_TRUE(rendererListener.mOverlayCallCount >= 1);
	/* sourceParams.session_meta was left zero-initialized: friendly_name
	 * starts out as an empty string. */
	CU_ASSERT_STRING_EQUAL(rendererListener.mFriendlyName, "");

	struct vmeta_session newMeta = {};
	strncpy(newMeta.friendly_name,
		"test-friendly-name",
		sizeof(newMeta.friendly_name) - 1);
	ret = source->setSessionMetadata(&newMeta);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	ret = renderer->render(&contentPos);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_STRING_EQUAL(rendererListener.mFriendlyName,
			       "test-friendly-name");

	CU_ASSERT_EQUAL(glGetError(), (GLenum)GL_NO_ERROR);

	rendererOwner.reset();
	sourceOwner.reset();

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
}


/* Shared by the three testCxxVideoRendererExtTexture*Dar* tests below:
 * pushes one frame, renders once, and checks the ext-texture size that
 * GlVideoRenderer::setupExtTexture() (pdraw_renderer_video_gl.cpp:1697-1823)
 * computed for it, via ExtTexCapturingRendererListener's loadVideoTexture()
 * capture. The source frame is always 64x64 with square pixels (SAR=1:1,
 * see makeRawVideoFrame()), which each caller's expectedWidth/expectedHeight
 * below were hand-computed against. */
static void
runExtTextureSizeTest(const struct pdraw_video_renderer_params &rendererParams,
		      unsigned int expectedWidth,
		      unsigned int expectedHeight)
{
	static constexpr uint32_t kWidth = 64;
	static constexpr uint32_t kHeight = 64;

	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	REQUIRE_GL_CONTEXT(glContext);

	struct pdraw_rect renderPos = {0, 0, kWidth, kHeight};
	ExtTexCapturingRendererListener rendererListener;
	IPdraw::IRawVideoSource *source = nullptr;
	IPdraw::IVideoRenderer *renderer = nullptr;
	setupRendererWithSource(loop,
				session,
				kWidth,
				kHeight,
				renderPos,
				rendererParams,
				mediaListener,
				rendererListener,
				&source,
				&renderer);
	auto rendererOwner = std::unique_ptr<IPdraw::IVideoRenderer>(renderer);
	auto sourceOwner = std::unique_ptr<IPdraw::IRawVideoSource>(source);

	pushFrameAndWaitRenderReady(
		loop, source, rendererListener, kWidth, kHeight, 0, 0);

	struct pdraw_rect contentPos = {};
	int ret = renderer->render(&contentPos);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_TRUE(contentPos.width > 0);

	CU_ASSERT_TRUE(rendererListener.mLoadCallCount >= 1);
	CU_ASSERT_EQUAL(rendererListener.mLoadedTextureWidth, expectedWidth);
	CU_ASSERT_EQUAL(rendererListener.mLoadedTextureHeight, expectedHeight);

	CU_ASSERT_EQUAL(glGetError(), (GLenum)GL_NO_ERROR);

	rendererOwner.reset();
	sourceOwner.reset();

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
}


/* "Custom DAR without custom texture width" branch (pdraw_renderer_video_gl.
 * cpp:1724-1751), never reached by testCxxVideoRendererExtTextureWithDAR
 * (which always sets video_texture_width too). dar=2/1=2.0 > ar=64/64=1.0,
 * so: width = height * dar_width / dar_height = 64 * 2 / 1 = 128,
 * height = source height = 64. */
static void testCxxVideoRendererExtTextureDarNoWidth()
{
	struct pdraw_video_renderer_params rendererParams = {};
	rendererParams.video_texture_dar_width = 2;
	rendererParams.video_texture_dar_height = 1;
	runExtTextureSizeTest(rendererParams, 128, 64);
}


/* "Custom texture width without custom DAR" branch (pdraw_renderer_video_gl.
 * cpp:1752-1765), never reached by testCxxVideoRendererExtTextureWithDAR
 * (which always sets a custom DAR too). No DAR override, so the height is
 * derived straight from the source's own resolution (128 * 64 / 64 = 128),
 * then adjusted for SAR (source SAR=1:1, so unchanged): width=128,
 * height=128. */
static void testCxxVideoRendererExtTextureWidthNoDar()
{
	struct pdraw_video_renderer_params rendererParams = {};
	rendererParams.video_texture_width = 128;
	runExtTextureSizeTest(rendererParams, 128, 128);
}


/* "No custom texture width and no custom DAR" branch (pdraw_renderer_video_gl
 * .cpp:1766-1788), never reached by any other test in this file (every other
 * ext-texture test sets at least one of the two params). With square pixels
 * (SAR=1:1) and no overrides, the texture is simply sized to the source's
 * own resolution: width=height=64. */
static void testCxxVideoRendererExtTextureNoWidthNoDar()
{
	struct pdraw_video_renderer_params rendererParams = {};
	runExtTextureSizeTest(rendererParams, 64, 64);
}


/* Exercises GlVideoRenderer::setNormalization() (pdraw_renderer_video_gl.cpp:
 * 1446-1499), called after every loadVideoFrame() -- never reached by any
 * other test in this file, all of which leave enable_auto_normalization at
 * its zero default.
 *
 * Confirmed by reading the source before writing this test: setNormalization()
 * only computes and applies real brightness/contrast coefficients (instead of
 * taking its "reset" path, brightness=0/contrast=1) when ALL of the following
 * hold:
 *   1. params->enable_auto_normalization is set.
 *   2. The *session*-level camera_spectrum (mMediaInfoSessionMeta, copied
 *      from the source's own session_meta) is VMETA_CAMERA_SPECTRUM_THERMAL
 *      -- set here via setupRendererWithSource()'s new sessionMeta param,
 *      applied to pdraw_video_source_params::session_meta at source-creation
 *      time.
 *   3. The frame's own per-frame metadata is present and of type
 *      VMETA_FRAME_TYPE_PROTO, with both thermal.min and thermal.max spots
 *      set (min.value < max.value) -- built here with the same writer API
 *      (vmeta_frame_proto_get_unpacked_rw/get_thermal/get_thermal_min/
 *      get_thermal_max) that libvideo-metadata's own vmeta_test_proto.c uses
 *      to construct synthetic thermal metadata for testing.
 * Neither mBrightnessCoef/mContrastCoef (GlVideo) nor mMediaInfoSessionMeta/
 * mNextFrame (GlVideoRenderer) have a public getter, so -- as with the
 * restart-flag and session-meta tests above -- this can only prove the code
 * runs to completion without crashing and that render() still succeeds, not
 * assert the exact resulting pixel output. */
static void testCxxVideoRendererAutoNormalizationThermal()
{
	static constexpr uint32_t kWidth = 64;
	static constexpr uint32_t kHeight = 64;

	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	REQUIRE_GL_CONTEXT(glContext);

	struct pdraw_rect renderPos = {0, 0, kWidth, kHeight};
	struct pdraw_video_renderer_params rendererParams = {};
	rendererParams.enable_auto_normalization = 1;

	struct vmeta_session sessionMeta = {};
	sessionMeta.camera_spectrum = VMETA_CAMERA_SPECTRUM_THERMAL;

	VideoRendererTrackingListener rendererListener;
	IPdraw::IRawVideoSource *source = nullptr;
	IPdraw::IVideoRenderer *renderer = nullptr;
	setupRendererWithSource(loop,
				session,
				kWidth,
				kHeight,
				renderPos,
				rendererParams,
				mediaListener,
				rendererListener,
				&source,
				&renderer,
				vdef_i420,
				8,
				nullptr,
				&sessionMeta);
	auto rendererOwner = std::unique_ptr<IPdraw::IVideoRenderer>(renderer);
	auto sourceOwner = std::unique_ptr<IPdraw::IRawVideoSource>(source);

	struct vmeta_frame *metadata = nullptr;
	int ret = vmeta_frame_new(VMETA_FRAME_TYPE_PROTO, &metadata);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	Vmeta__TimedMetadata *tm = nullptr;
	ret = vmeta_frame_proto_get_unpacked_rw(metadata, &tm);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	Vmeta__ThermalMetadata *thermal = vmeta_frame_proto_get_thermal(tm);
	CU_ASSERT_PTR_NOT_NULL_FATAL(thermal);
	Vmeta__ThermalSpot *minSpot =
		vmeta_frame_proto_get_thermal_min(thermal);
	CU_ASSERT_PTR_NOT_NULL_FATAL(minSpot);
	minSpot->value = 100;
	Vmeta__ThermalSpot *maxSpot =
		vmeta_frame_proto_get_thermal_max(thermal);
	CU_ASSERT_PTR_NOT_NULL_FATAL(maxSpot);
	maxSpot->value = 300;
	ret = vmeta_frame_proto_release_unpacked_rw(metadata, tm);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	pushFrameAndWaitRenderReady(loop,
				    source,
				    rendererListener,
				    kWidth,
				    kHeight,
				    0,
				    0,
				    vdef_i420,
				    8,
				    metadata);
	vmeta_frame_unref(metadata);

	struct pdraw_rect contentPos = {};
	ret = renderer->render(&contentPos);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_TRUE(contentPos.width > 0);

	CU_ASSERT_EQUAL(glGetError(), (GLenum)GL_NO_ERROR);

	rendererOwner.reset();
	sourceOwner.reset();

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
}


/* Same setup as testCxxVideoRendererAutoNormalizationThermal but without any
 * thermal metadata on the frame: exercises setNormalization()'s "reset" path
 * taken when the per-frame metadata check fails (mNextFrame.metadata is
 * nullptr), as opposed to the zero-initialized-params "reset" every other
 * test in this file already exercises implicitly. Distinct code path: this
 * one still has enable_auto_normalization=1 and camera_spectrum=THERMAL, so
 * it reaches the metadata check itself before bailing out, rather than
 * bailing out on the very first condition. */
static void testCxxVideoRendererAutoNormalizationThermalNoMetadataResets()
{
	static constexpr uint32_t kWidth = 64;
	static constexpr uint32_t kHeight = 64;

	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	REQUIRE_GL_CONTEXT(glContext);

	struct pdraw_rect renderPos = {0, 0, kWidth, kHeight};
	struct pdraw_video_renderer_params rendererParams = {};
	rendererParams.enable_auto_normalization = 1;

	struct vmeta_session sessionMeta = {};
	sessionMeta.camera_spectrum = VMETA_CAMERA_SPECTRUM_THERMAL;

	VideoRendererTrackingListener rendererListener;
	IPdraw::IRawVideoSource *source = nullptr;
	IPdraw::IVideoRenderer *renderer = nullptr;
	setupRendererWithSource(loop,
				session,
				kWidth,
				kHeight,
				renderPos,
				rendererParams,
				mediaListener,
				rendererListener,
				&source,
				&renderer,
				vdef_i420,
				8,
				nullptr,
				&sessionMeta);
	auto rendererOwner = std::unique_ptr<IPdraw::IVideoRenderer>(renderer);
	auto sourceOwner = std::unique_ptr<IPdraw::IRawVideoSource>(source);

	/* No metadata attached to this frame. */
	pushFrameAndWaitRenderReady(
		loop, source, rendererListener, kWidth, kHeight, 0, 0);

	struct pdraw_rect contentPos = {};
	int ret = renderer->render(&contentPos);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_TRUE(contentPos.width > 0);

	CU_ASSERT_EQUAL(glGetError(), (GLenum)GL_NO_ERROR);

	rendererOwner.reset();
	sourceOwner.reset();

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
}


/* Exercises GlVideoRenderer::setParams()'s PDRAW_VIDEO_RENDERER_DBG_FLAGS
 * env var parsing (pdraw_renderer_video_gl.cpp:2714-2730), never reached by
 * any other test in this file (none of them ever set this env var, so
 * getenv() always returned nullptr and dbgFlags stayed 0 everywhere else).
 *
 * Setting bit 0 (PDRAW_VIDEO_RENDERER_DBG_FLAG_MB_STATUS_OVERLAY,
 * pdraw_defs.h:98) makes setParams() set mMbStatusOverlay=true; that in turn
 * makes loadVideoFrame() look up the VSTRM_ANCILLARY_KEY_MB_STATUS ancillary
 * buffer on the loaded frame and forward it to GlVideo::loadFrame()
 * (pdraw_renderer_video_gl.cpp:1518-1526), which uploads it as a real GL
 * texture and sets mHasMbStatus=true (pdraw_gl_video.cpp:3239-3260) -- a
 * whole path no other test reaches, since none of them attach that
 * ancillary buffer either. The mb-status buffer is one byte per 16x16
 * macroblock (mbWidth=mbHeight=(64+15)/16=4 for this test's 64x64 frame,
 * pdraw_gl_video.cpp:3240-3241), so 16 bytes. */
static void testCxxVideoRendererMbStatusOverlayFromDbgFlagsEnv()
{
	static constexpr uint32_t kWidth = 64;
	static constexpr uint32_t kHeight = 64;

	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	REQUIRE_GL_CONTEXT(glContext);
	ScopedDbgFlagsEnv dbgFlagsEnv("1");

	struct pdraw_rect renderPos = {0, 0, kWidth, kHeight};
	struct pdraw_video_renderer_params rendererParams = {};
	VideoRendererTrackingListener rendererListener;
	IPdraw::IRawVideoSource *source = nullptr;
	IPdraw::IVideoRenderer *renderer = nullptr;
	setupRendererWithSource(loop,
				session,
				kWidth,
				kHeight,
				renderPos,
				rendererParams,
				mediaListener,
				rendererListener,
				&source,
				&renderer);
	auto rendererOwner = std::unique_ptr<IPdraw::IVideoRenderer>(renderer);
	auto sourceOwner = std::unique_ptr<IPdraw::IRawVideoSource>(source);

	struct mbuf_raw_video_frame_queue *inQueue = source->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(inQueue);
	struct mbuf_raw_video_frame *frame =
		makeRawVideoFrame(vdef_i420, 8, kWidth, kHeight, 0, 0);
	uint8_t mbStatus[16] = {};
	int ret = mbuf_raw_video_frame_add_ancillary_buffer(
		frame,
		VSTRM_ANCILLARY_KEY_MB_STATUS,
		mbStatus,
		sizeof(mbStatus));
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	ret = mbuf_raw_video_frame_queue_push(inQueue, frame);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	mbuf_raw_video_frame_unref(frame);

	int countBefore = rendererListener.mRenderReadyCount.load();
	bool gotRenderReady = loop.pumpUntil(
		[&]() {
			return rendererListener.mRenderReadyCount.load() >
			       countBefore;
		},
		15000);
	CU_ASSERT_TRUE_FATAL(gotRenderReady);

	struct pdraw_rect contentPos = {};
	ret = renderer->render(&contentPos);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_TRUE(contentPos.width > 0);

	CU_ASSERT_EQUAL(glGetError(), (GLenum)GL_NO_ERROR);

	rendererOwner.reset();
	sourceOwner.reset();

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
}


/* Same env var mechanism as testCxxVideoRendererMbStatusOverlayFromDbgFlags
 * Env, but with a value strtol() cannot parse as a non-negative integer:
 * exercises setParams()'s malformed-input guard (pdraw_renderer_video_gl.cpp:
 * 2716-2727, the "endptr[0] != '\0' || parsedint < 0 || errno != 0" branch),
 * which no other test reaches either. setParams() tolerates this (logs and
 * leaves dbgFlags=0) rather than failing, so the only observable proof is
 * that setParams()/render() keep succeeding instead of malfunctioning. */
static void testCxxVideoRendererMalformedDbgFlagsEnvIgnored()
{
	static constexpr uint32_t kWidth = 64;
	static constexpr uint32_t kHeight = 64;

	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	REQUIRE_GL_CONTEXT(glContext);
	ScopedDbgFlagsEnv dbgFlagsEnv("not-a-number");

	struct pdraw_rect renderPos = {0, 0, kWidth, kHeight};
	struct pdraw_video_renderer_params rendererParams = {};
	VideoRendererTrackingListener rendererListener;
	IPdraw::IRawVideoSource *source = nullptr;
	IPdraw::IVideoRenderer *renderer = nullptr;
	setupRendererWithSource(loop,
				session,
				kWidth,
				kHeight,
				renderPos,
				rendererParams,
				mediaListener,
				rendererListener,
				&source,
				&renderer);
	auto rendererOwner = std::unique_ptr<IPdraw::IVideoRenderer>(renderer);
	auto sourceOwner = std::unique_ptr<IPdraw::IRawVideoSource>(source);

	/* setParams() re-reads the env var on every call, not just at
	 * construction time -- call it again explicitly to also cover that
	 * call site (in addition to the constructor's own internal call). */
	struct pdraw_video_renderer_params currentParams = {};
	int ret = renderer->getParams(&currentParams);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	ret = renderer->setParams(&currentParams);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	pushFrameAndWaitRenderReady(
		loop, source, rendererListener, kWidth, kHeight, 0, 0);

	struct pdraw_rect contentPos = {};
	ret = renderer->render(&contentPos);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_TRUE(contentPos.width > 0);

	CU_ASSERT_EQUAL(glGetError(), (GLenum)GL_NO_ERROR);

	rendererOwner.reset();
	sourceOwner.reset();

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
}


/* Exercises GlVideoRenderer::onVideoPresStatsTimer() (pdraw_renderer_video_gl
 * .cpp:691-715) by letting its real pomp::Timer actually fire, rather than
 * calling it directly -- no other test in this file ever waits long enough
 * for it to. The timer is armed for the first time inside render()'s
 * "mFirstFrame" block (pdraw_renderer_video_gl.cpp:2349-2355), with both an
 * initial delay and a repeat period of GL_RENDERER_VIDEO_PRES_STATS_TIME_MS
 * = 200ms, so one priming render() plus pumping the loop for longer than
 * that is enough to guarantee at least one real firing.
 *
 * onVideoPresStatsTimer() calls Channel::sendVideoPresStats(), which is a
 * real (not stubbed) upstream-event send: it serializes mVideoPresStats into
 * a pomp::Message and hands it to the channel's source listener
 * (Channel::sendVideoPresStats(), pdraw_channel.cpp:310-326) -- genuinely
 * exercised here for the raw-video-source pipeline, unlike
 * test_pipeline_demuxer_stream.cpp's coverage of the same mechanism for
 * stream demuxer channels.
 *
 * Caveat confirmed by reading the source before writing this test:
 * ExternalRawVideoSource never overrides Source::onChannelVideoPresStats()
 * (only onChannelFlushed/onChannelDrained are overridden, pdraw_external_raw
 * _video_source.hpp:93-95), so the base class's no-op runs and there is no
 * IRawVideoSource::Listener callback this reaches -- same "no public
 * observable" limitation as the auto-normalization and session-meta tests
 * above. This test can only prove the timer fires and the object keeps
 * working afterward, not observe sendVideoPresStats()'s effect. */
static void testCxxVideoRendererVideoPresStatsTimerFires()
{
	static constexpr uint32_t kWidth = 64;
	static constexpr uint32_t kHeight = 64;

	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	REQUIRE_GL_CONTEXT(glContext);

	struct pdraw_rect renderPos = {0, 0, kWidth, kHeight};
	struct pdraw_video_renderer_params rendererParams = {};
	VideoRendererTrackingListener rendererListener;
	IPdraw::IRawVideoSource *source = nullptr;
	IPdraw::IVideoRenderer *renderer = nullptr;
	setupRendererWithSource(loop,
				session,
				kWidth,
				kHeight,
				renderPos,
				rendererParams,
				mediaListener,
				rendererListener,
				&source,
				&renderer);
	auto rendererOwner = std::unique_ptr<IPdraw::IVideoRenderer>(renderer);
	auto sourceOwner = std::unique_ptr<IPdraw::IRawVideoSource>(source);

	pushFrameAndWaitRenderReady(
		loop, source, rendererListener, kWidth, kHeight, 0, 0);

	/* Arms mVideoPresStatsTimer (mFirstFrame block). */
	struct pdraw_rect contentPos = {};
	int ret = renderer->render(&contentPos);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_TRUE_FATAL(contentPos.width > 0);

	/* Let the real 200ms-period timer fire at least once; the predicate
	 * is deliberately never true, this just pumps the loop for the
	 * bounded real-time window so the pomp::Timer's own fd wakes it up. */
	(void)loop.pumpUntil([]() { return false; }, 500);

	/* The renderer must still work normally after the timer fired. */
	ret = renderer->render(&contentPos);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_TRUE(contentPos.width > 0);

	CU_ASSERT_EQUAL(glGetError(), (GLenum)GL_NO_ERROR);

	rendererOwner.reset();
	sourceOwner.reset();

	int stopRet = session->stop();
	CU_ASSERT_EQUAL(stopRet, 0);
	bool gotStop = loop.pumpUntil(
		[&mediaListener]() { return mediaListener.mGotStopResponse; },
		20000);
	CU_ASSERT_TRUE(gotStop);
}


/* ── C API: PdrawVideoRendererListener shim coverage ─────────────────────
 * Covers all 5 virtual methods of PdrawVideoRendererListener
 * (pdraw_wrapper.cpp) via the C struct pdraw_video_renderer_cbs: media_added,
 * media_removed, render_ready, load_texture, render_overlay. Uses the same
 * offscreen EGL context (TestGlContext) as the C++ tests above — required by
 * pdraw_video_renderer_new() whose constructor calls glGetIntegerv. */

namespace {

struct RendSessionCbState {
	unsigned int lastRawMediaId = 0;
};

struct RendCbState {
	int mediaAddedCount = 0;
	int mediaRemovedCount = 0;
	int mediaRemovedRestart = -1;
	int renderReadyCount = 0;
	int loadTextureCount = 0;
	int renderOverlayCount = 0;
};

} /* anonymous namespace */

static void
rend_session_stop_cb(struct pdraw * /*p*/, int /*status*/, void * /*ud*/)
{
}

static void rend_session_media_added_cb(struct pdraw * /*p*/,
					const struct pdraw_media_info *info,
					void * /*elem*/,
					void *ud)
{
	if (info->type == PDRAW_MEDIA_TYPE_VIDEO &&
	    info->video.format == VDEF_FRAME_TYPE_RAW)
		static_cast<RendSessionCbState *>(ud)->lastRawMediaId =
			info->id;
}

static void
rend_session_media_removed_cb(struct pdraw * /*p*/,
			      const struct pdraw_media_info * /*info*/,
			      void * /*elem*/,
			      void * /*ud*/)
{
}

static void rend_src_flushed_cb(struct pdraw * /*p*/,
				struct pdraw_raw_video_source * /*s*/,
				void * /*ud*/)
{
}

static void rend_src_drained_cb(struct pdraw * /*p*/,
				struct pdraw_raw_video_source * /*s*/,
				void * /*ud*/)
{
}

static void rend_c_media_added_cb(struct pdraw * /*p*/,
				  struct pdraw_video_renderer * /*r*/,
				  const struct pdraw_media_info * /*info*/,
				  void *ud)
{
	static_cast<RendCbState *>(ud)->mediaAddedCount++;
}

static void rend_c_media_removed_cb(struct pdraw * /*p*/,
				    struct pdraw_video_renderer * /*r*/,
				    const struct pdraw_media_info * /*info*/,
				    int restart,
				    void *ud)
{
	auto *s = static_cast<RendCbState *>(ud);
	s->mediaRemovedRestart = restart;
	s->mediaRemovedCount++;
}

static void rend_c_render_ready_cb(struct pdraw * /*p*/,
				   struct pdraw_video_renderer * /*r*/,
				   void *ud)
{
	static_cast<RendCbState *>(ud)->renderReadyCount++;
}

static int rend_c_load_texture_cb(struct pdraw * /*p*/,
				  struct pdraw_video_renderer * /*r*/,
				  unsigned int tw,
				  unsigned int /*th*/,
				  const struct pdraw_media_info * /*info*/,
				  struct mbuf_raw_video_frame * /*frame*/,
				  const void * /*fudata*/,
				  size_t /*fudatalen*/,
				  void *ud)
{
	/* Probe at GlVideoRenderer constructor time passes tw==0; real render
	 * calls pass tw>0 after the renderer computes the texture dimensions.
	 */
	if (tw > 0)
		static_cast<RendCbState *>(ud)->loadTextureCount++;
	return 0; /* 0 → ext-texture path (mRenderExternalVideo=true) */
}

static void
rend_c_render_overlay_cb(struct pdraw * /*p*/,
			 struct pdraw_video_renderer * /*r*/,
			 const struct pdraw_rect * /*rpos*/,
			 const struct pdraw_rect * /*cpos*/,
			 const float * /*vm*/,
			 const float * /*pm*/,
			 const struct pdraw_media_info * /*info*/,
			 struct vmeta_frame * /*meta*/,
			 const struct pdraw_video_frame_extra * /*extra*/,
			 void *ud)
{
	static_cast<RendCbState *>(ud)->renderOverlayCount++;
}

static void testCVideoRendererListenerCallbacks()
{
	static constexpr uint32_t kWidth = 64;
	static constexpr uint32_t kHeight = 64;

	TestPompLoop loop;
	RendSessionCbState sessState;
	struct pdraw_cbs sessCbs = {};
	sessCbs.stop_resp = rend_session_stop_cb;
	sessCbs.media_added = rend_session_media_added_cb;
	sessCbs.media_removed = rend_session_media_removed_cb;
	struct pdraw *p = nullptr;
	int ret = pdraw_new(loop.raw(), &sessCbs, &sessState, &p);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(p);

	/* GL context must be current before pdraw_video_renderer_new():
	 * GlVideoRenderer::setup() calls glGetIntegerv synchronously. */
	REQUIRE_GL_CONTEXT(glContext);

	/* Raw video source (I420 64×64, live). flushed/drained are mandatory
	 * in pdraw_raw_video_source_cbs (wrapper returns -EINVAL if null). */
	struct pdraw_video_source_params srcParams = {};
	srcParams.playback_type = PDRAW_PLAYBACK_TYPE_LIVE;
	srcParams.video.format = VDEF_FRAME_TYPE_RAW;
	srcParams.video.raw.format = vdef_i420;
	srcParams.video.raw.info.resolution.width = kWidth;
	srcParams.video.raw.info.resolution.height = kHeight;
	srcParams.video.raw.info.bit_depth = 8;
	srcParams.video.raw.info.framerate.num = 30;
	srcParams.video.raw.info.framerate.den = 1;
	struct pdraw_raw_video_source_cbs srcCbs = {};
	srcCbs.flushed = rend_src_flushed_cb;
	srcCbs.drained = rend_src_drained_cb;
	struct pdraw_raw_video_source *src = nullptr;
	ret = pdraw_raw_video_source_new(p, &srcParams, &srcCbs, nullptr, &src);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(src);

	/* Wait for the session-level media_added to learn the raw media ID */
	bool gotMedia = loop.pumpUntil(
		[&sessState]() { return sessState.lastRawMediaId != 0; },
		15000);
	CU_ASSERT_TRUE_FATAL(gotMedia);

	/* Create renderer via C API on the live raw media.
	 * load_texture returning 0 activates the ext-texture path at probe
	 * time (mRenderExternalVideo=true). render_overlay non-null makes the
	 * wrapper return 0 at probe time (mRenderVideoOverlay=true). */
	struct pdraw_rect renderPos = {0, 0, kWidth, kHeight};
	struct pdraw_video_renderer_params rendParams = {};
	RendCbState rendState;
	struct pdraw_video_renderer_cbs rendCbs = {};
	rendCbs.media_added = rend_c_media_added_cb;
	rendCbs.media_removed = rend_c_media_removed_cb;
	rendCbs.render_ready = rend_c_render_ready_cb;
	rendCbs.load_texture = rend_c_load_texture_cb;
	rendCbs.render_overlay = rend_c_render_overlay_cb;
	struct pdraw_video_renderer *rend = nullptr;
	ret = pdraw_video_renderer_new(p,
				       sessState.lastRawMediaId,
				       &renderPos,
				       &rendParams,
				       &rendCbs,
				       &rendState,
				       &rend);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(rend);

	/* media_added: fired on the pomp thread when the renderer connects */
	bool gotRendMedia = loop.pumpUntil(
		[&rendState]() { return rendState.mediaAddedCount >= 1; },
		15000);
	CU_ASSERT_TRUE(gotRendMedia);
	CU_ASSERT_EQUAL(rendState.mediaAddedCount, 1);

	/* render_ready: push a frame → GlVideoRenderer::queueEventCb fires
	 * onVideoRenderReady synchronously from the pomp thread (ASAP mode) */
	struct mbuf_raw_video_frame_queue *inQ =
		pdraw_raw_video_source_get_queue(p, src);
	CU_ASSERT_PTR_NOT_NULL_FATAL(inQ);
	struct mbuf_raw_video_frame *frame =
		makeRawVideoFrame(vdef_i420, 8, kWidth, kHeight, 0, 0);
	ret = mbuf_raw_video_frame_queue_push(inQ, frame);
	CU_ASSERT_EQUAL(ret, 0);
	mbuf_raw_video_frame_unref(frame);

	bool gotRenderReady = loop.pumpUntil(
		[&rendState]() { return rendState.renderReadyCount >= 1; },
		15000);
	CU_ASSERT_TRUE(gotRenderReady);

	/* load_texture + render_overlay: both fired by render() from the
	 * rendering thread (same thread here — single-threaded test).
	 * load_texture is called before the frame upload (ext-texture path);
	 * render_overlay is called after the frame is drawn. */
	struct pdraw_rect contentPos = {};
	ret = pdraw_video_renderer_render(p, rend, &contentPos);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_TRUE(rendState.loadTextureCount >= 1);
	CU_ASSERT_TRUE(rendState.renderOverlayCount >= 1);

	/* media_removed: fired (pomp thread) when the source is destroyed →
	 * session withdraws the raw media from the renderer.
	 * No prior reconfigure/resolution/framerate change → restart=0. */
	pdraw_raw_video_source_destroy(p, src);
	src = nullptr;
	bool gotRemoved = loop.pumpUntil(
		[&rendState]() { return rendState.mediaRemovedCount >= 1; },
		15000);
	CU_ASSERT_TRUE(gotRemoved);
	CU_ASSERT_EQUAL(rendState.mediaRemovedCount, 1);
	CU_ASSERT_EQUAL(rendState.mediaRemovedRestart, 0);

	pdraw_video_renderer_destroy(p, rend);
	pdraw_destroy(p);
}


static void testCxxVideoRendererSchedulingModes()
{
	REQUIRE_GL_CONTEXT(glCtx);
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	struct pdraw_rect renderPos = {0, 0, 64, 48};
	struct pdraw_video_renderer_params params = {};
	params.scheduling_mode = PDRAW_VIDEO_RENDERER_SCHEDULING_MODE_ADAPTIVE;

	IPdraw::IRawVideoSource *source = nullptr;
	VideoRendererTrackingListener rendererListener;
	IPdraw::IVideoRenderer *renderer = nullptr;
	setupRendererWithSource(loop,
				session,
				64,
				48,
				renderPos,
				params,
				mediaListener,
				rendererListener,
				&source,
				&renderer);
	auto sourceOwner = std::unique_ptr<IPdraw::IRawVideoSource>(source);
	auto rendererOwner = std::unique_ptr<IPdraw::IVideoRenderer>(renderer);

	pushFrameAndWaitRenderReady(
		loop, source, rendererListener, 64, 48, 0, 0);

	struct pdraw_rect contentPos = {};
	int ret = renderer->render(&contentPos);
	CU_ASSERT_EQUAL(ret, 0);

	stopSessionAndWait(&loop, session, &mediaListener);

	rendererOwner.reset();
	sourceOwner.reset();
}


static void testCxxVideoRendererSimplifiedRendering()
{
	REQUIRE_GL_CONTEXT(glCtx);
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	struct pdraw_rect renderPos = {0, 0, 64, 48};
	struct pdraw_video_renderer_params params = {};
	params.enable_simplified_rendering = 1;

	IPdraw::IRawVideoSource *source = nullptr;
	VideoRendererTrackingListener rendererListener;
	IPdraw::IVideoRenderer *renderer = nullptr;
	setupRendererWithSource(loop,
				session,
				64,
				48,
				renderPos,
				params,
				mediaListener,
				rendererListener,
				&source,
				&renderer);
	auto sourceOwner = std::unique_ptr<IPdraw::IRawVideoSource>(source);
	auto rendererOwner = std::unique_ptr<IPdraw::IVideoRenderer>(renderer);

	pushFrameAndWaitRenderReady(
		loop, source, rendererListener, 64, 48, 0, 0);

	struct pdraw_rect contentPos = {};
	int ret = renderer->render(&contentPos);
	CU_ASSERT_EQUAL(ret, 0);

	stopSessionAndWait(&loop, session, &mediaListener);

	rendererOwner.reset();
	sourceOwner.reset();
}


static void testCxxVideoRendererFillModeFitAndCropContentPos()
{
	REQUIRE_GL_CONTEXT(glCtx);
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	struct pdraw_rect renderPos = {0, 0, 1000, 1000};
	struct pdraw_video_renderer_params params = {};
	params.fill_mode = PDRAW_VIDEO_RENDERER_FILL_MODE_FIT;

	IPdraw::IRawVideoSource *source = nullptr;
	VideoRendererTrackingListener rendererListener;
	IPdraw::IVideoRenderer *renderer = nullptr;
	setupRendererWithSource(loop,
				session,
				1280,
				720,
				renderPos,
				params,
				mediaListener,
				rendererListener,
				&source,
				&renderer);
	auto sourceOwner = std::unique_ptr<IPdraw::IRawVideoSource>(source);
	auto rendererOwner = std::unique_ptr<IPdraw::IVideoRenderer>(renderer);

	pushFrameAndWaitRenderReady(
		loop, source, rendererListener, 1280, 720, 0, 0);

	/* FIT mode: letterboxing */
	struct pdraw_rect contentPos = {};
	int ret = renderer->render(&contentPos);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(contentPos.x, 0);
	CU_ASSERT_EQUAL(contentPos.width, 1000);
	CU_ASSERT_TRUE(contentPos.height > 550 && contentPos.height < 570);

	/* Switch to CROP mode */
	params.fill_mode = PDRAW_VIDEO_RENDERER_FILL_MODE_CROP;
	ret = renderer->setParams(&params);
	CU_ASSERT_EQUAL(ret, 0);

	ret = renderer->render(&contentPos);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(contentPos.y, 0);
	CU_ASSERT_EQUAL(contentPos.height, 1000);
	CU_ASSERT_TRUE(contentPos.width > 1700);

	stopSessionAndWait(&loop, session, &mediaListener);

	rendererOwner.reset();
	sourceOwner.reset();
}


static void testCxxVideoRendererEnableVerticalMirror()
{
	REQUIRE_GL_CONTEXT(glCtx);
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	struct pdraw_rect renderPos = {0, 0, 64, 48};
	struct pdraw_video_renderer_params params = {};
	params.vertical_mirror = true;

	IPdraw::IRawVideoSource *source = nullptr;
	VideoRendererTrackingListener rendererListener;
	IPdraw::IVideoRenderer *renderer = nullptr;
	setupRendererWithSource(loop,
				session,
				64,
				48,
				renderPos,
				params,
				mediaListener,
				rendererListener,
				&source,
				&renderer);
	auto sourceOwner = std::unique_ptr<IPdraw::IRawVideoSource>(source);
	auto rendererOwner = std::unique_ptr<IPdraw::IVideoRenderer>(renderer);

	pushFrameAndWaitRenderReady(
		loop, source, rendererListener, 64, 48, 0, 0);

	struct pdraw_rect contentPos = {};
	int ret = renderer->render(&contentPos);
	CU_ASSERT_EQUAL(ret, 0);

	stopSessionAndWait(&loop, session, &mediaListener);

	rendererOwner.reset();
	sourceOwner.reset();
}


static void testCxxVideoRendererResizeInvalidArgs()
{
	REQUIRE_GL_CONTEXT(glCtx);
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	struct pdraw_rect renderPos = {0, 0, 64, 48};
	struct pdraw_video_renderer_params params = {};

	IPdraw::IRawVideoSource *source = nullptr;
	VideoRendererTrackingListener rendererListener;
	IPdraw::IVideoRenderer *renderer = nullptr;
	setupRendererWithSource(loop,
				session,
				64,
				48,
				renderPos,
				params,
				mediaListener,
				rendererListener,
				&source,
				&renderer);
	auto sourceOwner = std::unique_ptr<IPdraw::IRawVideoSource>(source);
	auto rendererOwner = std::unique_ptr<IPdraw::IVideoRenderer>(renderer);

	/* resize(nullptr) -> -EINVAL */
	int ret = renderer->resize(nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* resize with zero dimensions -> -EPROTO */
	struct pdraw_rect zeroPos = {0, 0, 0, 0};
	ret = renderer->resize(&zeroPos);
	CU_ASSERT_EQUAL(ret, -EPROTO);

	stopSessionAndWait(&loop, session, &mediaListener);

	rendererOwner.reset();
	sourceOwner.reset();
}


static void testCxxVideoRendererCustomViewAndProjMatrix()
{
	REQUIRE_GL_CONTEXT(glCtx);
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	struct pdraw_rect renderPos = {0, 0, 64, 48};
	struct pdraw_video_renderer_params params = {};

	IPdraw::IRawVideoSource *source = nullptr;
	VideoRendererTrackingListener rendererListener;
	IPdraw::IVideoRenderer *renderer = nullptr;
	setupRendererWithSource(loop,
				session,
				64,
				48,
				renderPos,
				params,
				mediaListener,
				rendererListener,
				&source,
				&renderer);
	auto sourceOwner = std::unique_ptr<IPdraw::IRawVideoSource>(source);
	auto rendererOwner = std::unique_ptr<IPdraw::IVideoRenderer>(renderer);

	pushFrameAndWaitRenderReady(
		loop, source, rendererListener, 64, 48, 0, 0);

	/* 4x4 identity matrices */
	static const float identityMat[16] = {
		1.0f,
		0.0f,
		0.0f,
		0.0f,
		0.0f,
		1.0f,
		0.0f,
		0.0f,
		0.0f,
		0.0f,
		1.0f,
		0.0f,
		0.0f,
		0.0f,
		0.0f,
		1.0f,
	};

	struct pdraw_rect contentPos = {};
	int ret = renderer->render(&contentPos, identityMat, identityMat);
	CU_ASSERT_EQUAL(ret, 0);

	stopSessionAndWait(&loop, session, &mediaListener);

	rendererOwner.reset();
	sourceOwner.reset();
}


/* Exercises setupPaddingFbo()'s width > height branch
 * (pdraw_gl_video.cpp:1827-1836) and renderPadding()'s FIT_PAD_BLUR_CROP branch
 * (pdraw_gl_video.cpp:2304-2312) using a wide video (128x64, width > height)
 * rendered into a square viewport (64x64). */
static void testCxxVideoRendererFillModePadBlurCropWide()
{
	static constexpr uint32_t kWidth = 128;
	static constexpr uint32_t kHeight = 64;

	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	REQUIRE_GL_CONTEXT(glContext);

	struct pdraw_rect renderPos = {0, 0, 64, 64};
	struct pdraw_video_renderer_params rendererParams = {};
	rendererParams.fill_mode =
		PDRAW_VIDEO_RENDERER_FILL_MODE_FIT_PAD_BLUR_CROP;

	VideoRendererTrackingListener rendererListener;
	IPdraw::IRawVideoSource *source = nullptr;
	IPdraw::IVideoRenderer *renderer = nullptr;
	setupRendererWithSource(loop,
				session,
				kWidth,
				kHeight,
				renderPos,
				rendererParams,
				mediaListener,
				rendererListener,
				&source,
				&renderer);
	auto rendererOwner = std::unique_ptr<IPdraw::IVideoRenderer>(renderer);
	auto sourceOwner = std::unique_ptr<IPdraw::IRawVideoSource>(source);

	pushFrameAndWaitRenderReady(
		loop, source, rendererListener, kWidth, kHeight, 0, 0);

	struct pdraw_rect contentPos = {};
	int ret = renderer->render(&contentPos);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_TRUE(contentPos.width > 0);

	stopSessionAndWait(&loop, session, &mediaListener);

	rendererOwner.reset();
	sourceOwner.reset();
}


static void testCxxVideoRendererAdaptiveSchedulingFullCoverage()
{
	REQUIRE_GL_CONTEXT(glCtx);
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	struct pdraw_rect renderPos = {0, 0, 64, 48};
	struct pdraw_video_renderer_params params = {};
	params.scheduling_mode = PDRAW_VIDEO_RENDERER_SCHEDULING_MODE_ADAPTIVE;

	IPdraw::IRawVideoSource *source = nullptr;
	VideoRendererTrackingListener rendererListener;
	IPdraw::IVideoRenderer *renderer = nullptr;
	setupRendererWithSource(loop,
				session,
				64,
				48,
				renderPos,
				params,
				mediaListener,
				rendererListener,
				&source,
				&renderer);
	auto sourceOwner = std::unique_ptr<IPdraw::IRawVideoSource>(source);
	auto rendererOwner = std::unique_ptr<IPdraw::IVideoRenderer>(renderer);

	struct mbuf_raw_video_frame_queue *queue = source->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(queue);
	struct pdraw_rect contentPos = {};

	/* Step 1: Push 4 frames (4 > 5/2 = 2.5 and 4 <= 5 max_frames)
	 * with 1GHz timescale (so frameTs delta in us is 0 -> timingError <= 0,
	 * preventing early frame breaks and draining all 4 frames in 1 render
	 * call) */
	for (unsigned int i = 0; i < 4; i++) {
		struct mbuf_raw_video_frame *frame =
			makeRawVideoFrame(vdef_i420,
					  8,
					  64,
					  48,
					  (i + 1) * 10,
					  i,
					  nullptr,
					  1000000000);
		int ret = mbuf_raw_video_frame_queue_push(queue, frame);
		CU_ASSERT_EQUAL(ret, 0);
		mbuf_raw_video_frame_unref(frame);
	}

	/* Pump loop so all 4 frames transfer to GlVideoRenderer queue */
	bool got4 = loop.pumpUntil(
		[&rendererListener]() {
			return rendererListener.mRenderReadyCount.load() >= 4;
		},
		1000);
	CU_ASSERT_TRUE(got4);

	/* render() sees count=4 > 5/2 -> exits initial buffering (l.
	 * 1946-1952),
	 * and drains all 4 frames (timingError <= 0), queue becomes empty! */
	int ret = renderer->render(&contentPos);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_TRUE(contentPos.width > 0);
	CU_ASSERT_TRUE(contentPos.height > 0);

	/* Step 2: Render on EMPTY QUEUE when !mSchedInitialBuffering (l.
	 * 2074-2083) queue is now empty (count=0) and mSchedInitialBuffering is
	 * false -> resets mSchedInitialBuffering = true (l. 2074-2083)! */
	ret = renderer->render(&contentPos);
	CU_ASSERT_EQUAL(ret, 0);

	/* Step 3: Queue Full Discard (l. 1860-1878)
	 * Push 5 frames into source queue with timestamps > 40 (100..104) */
	for (unsigned int i = 0; i < 5; i++) {
		struct mbuf_raw_video_frame *frame =
			makeRawVideoFrame(vdef_i420,
					  8,
					  64,
					  48,
					  (100 + i) * 10,
					  10 + i,
					  nullptr,
					  1000000000);
		int retPush = mbuf_raw_video_frame_queue_push(queue, frame);
		CU_ASSERT_EQUAL(retPush, 0);
		mbuf_raw_video_frame_unref(frame);
	}

	/* Pump loop so all 5 frames reach GlVideoRenderer queue
	 * (mRenderReadyCount reaches 4 + 5 = 9) */
	bool got9 = loop.pumpUntil(
		[&rendererListener]() {
			return rendererListener.mRenderReadyCount.load() >= 9;
		},
		1000);
	CU_ASSERT_TRUE(got9);

	/* render() sees count=5 >= 5 -> pops & discards oldest frame (l.
	 * 1860-1878)! */
	ret = renderer->render(&contentPos);
	CU_ASSERT_EQUAL(ret, 0);

	/* Step 4: Early frame & Late frame branches (l. 1968 & 2007) */
	struct mbuf_raw_video_frame *earlyFrame = makeRawVideoFrame(
		vdef_i420, 8, 64, 48, 1000000000ULL, 100, nullptr, 1000000000);
	ret = mbuf_raw_video_frame_queue_push(queue, earlyFrame);
	CU_ASSERT_EQUAL(ret, 0);
	mbuf_raw_video_frame_unref(earlyFrame);

	bool got10 = loop.pumpUntil(
		[&rendererListener]() {
			return rendererListener.mRenderReadyCount.load() >= 10;
		},
		1000);
	CU_ASSERT_TRUE(got10);

	ret = renderer->render(&contentPos);
	CU_ASSERT_EQUAL(ret, 0);

	struct mbuf_raw_video_frame *lateFrame = makeRawVideoFrame(
		vdef_i420, 8, 64, 48, 1000000001ULL, 101, nullptr, 1000000000);
	ret = mbuf_raw_video_frame_queue_push(queue, lateFrame);
	CU_ASSERT_EQUAL(ret, 0);
	mbuf_raw_video_frame_unref(lateFrame);

	bool got11 = loop.pumpUntil(
		[&rendererListener]() {
			return rendererListener.mRenderReadyCount.load() >= 11;
		},
		1000);
	CU_ASSERT_TRUE(got11);

	ret = renderer->render(&contentPos);
	CU_ASSERT_EQUAL(ret, 0);

	/* Step 5: Early frame with queue count >= 4 -> triggers _processAnyway
	 * (l. 1978-1987) */
	for (unsigned int i = 0; i < 4; i++) {
		struct mbuf_raw_video_frame *frame = makeRawVideoFrame(
			vdef_i420,
			8,
			64,
			48,
			2000000000ULL + (i + 1) * 1000000000ULL,
			200 + i,
			nullptr,
			1000000);
		int retPush = mbuf_raw_video_frame_queue_push(queue, frame);
		CU_ASSERT_EQUAL(retPush, 0);
		mbuf_raw_video_frame_unref(frame);
	}

	bool got15 = loop.pumpUntil(
		[&rendererListener]() {
			return rendererListener.mRenderReadyCount.load() >= 15;
		},
		1000);
	CU_ASSERT_TRUE(got15);

	/* First render call leaves 3 early frames in queue */
	ret = renderer->render(&contentPos);
	CU_ASSERT_EQUAL(ret, 0);

	/* Push 1 more early frame so queue count reaches 4 (>=
	 * GL_RENDERER_QUEUE_MAX_FRAMES - 1) */
	struct mbuf_raw_video_frame *extraEarlyFrame = makeRawVideoFrame(
		vdef_i420, 8, 64, 48, 7000000000ULL, 204, nullptr, 1000000);
	ret = mbuf_raw_video_frame_queue_push(queue, extraEarlyFrame);
	CU_ASSERT_EQUAL(ret, 0);
	mbuf_raw_video_frame_unref(extraEarlyFrame);

	bool got16 = loop.pumpUntil(
		[&rendererListener]() {
			return rendererListener.mRenderReadyCount.load() >= 16;
		},
		1000);
	CU_ASSERT_TRUE(got16);

	/* render() sees count=4 >= 4 and early frame -> triggers _processAnyway
	 * = true (l. 1978-1987)! */
	ret = renderer->render(&contentPos);
	CU_ASSERT_EQUAL(ret, 0);

	stopSessionAndWait(&loop, session, &mediaListener);

	rendererOwner.reset();
	sourceOwner.reset();
}


static void testCxxVideoRendererAsyncDrainOnFlush()
{
	REQUIRE_GL_CONTEXT(glCtx);
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	struct pdraw_rect renderPos = {0, 0, 64, 48};
	struct pdraw_video_renderer_params params = {};

	IPdraw::IRawVideoSource *source = nullptr;
	VideoRendererTrackingListener rendererListener;
	IPdraw::IVideoRenderer *renderer = nullptr;
	setupRendererWithSource(loop,
				session,
				64,
				48,
				renderPos,
				params,
				mediaListener,
				rendererListener,
				&source,
				&renderer);
	auto sourceOwner = std::unique_ptr<IPdraw::IRawVideoSource>(source);
	auto rendererOwner = std::unique_ptr<IPdraw::IVideoRenderer>(renderer);

	struct mbuf_raw_video_frame_queue *queue = source->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(queue);
	struct pdraw_rect contentPos = {};

	/* Push 2 frames and wait until they reach GlVideoRenderer's input queue
	 */
	pushFrameAndWaitRenderReady(
		loop, source, rendererListener, 64, 48, 10, 0);
	pushFrameAndWaitRenderReady(
		loop, source, rendererListener, 64, 48, 20, 1);

	/* Trigger async drain on the source */
	int ret = source->drain();
	CU_ASSERT_EQUAL(ret, 0);

	/* Pump loop until GlVideoRenderer receives FLUSH event and enters
	 * FLUSHING state */
	auto *wrapper = dynamic_cast<VideoRendererWrapper *>(renderer);
	CU_ASSERT_PTR_NOT_NULL_FATAL(wrapper);
	Element *el = wrapper->getElement();
	CU_ASSERT_PTR_NOT_NULL_FATAL(el);

	bool gotFlushing = loop.pumpUntil(
		[el]() {
			return el->getFlushingState() ==
			       Element::FlushingState::FLUSHING;
		},
		1000);
	CU_ASSERT_TRUE(gotFlushing);

	/* Call render() while FlushingState is FLUSHING and queue count becomes
	 * 0! scheduleFrame() drains remaining frames, sees count==0 and
	 * FlushingState::FLUSHING, and registers mIdleCompleteDrainHandler on
	 * pomp loop (l. 2131-2140)! */
	ret = renderer->render(&contentPos);
	CU_ASSERT_EQUAL(ret, 0);

	/* Pump loop until mIdleCompleteDrainHandler fires and sets
	 * FlushingState to FLUSHED */
	bool gotFlushed = loop.pumpUntil(
		[el]() {
			return el->getFlushingState() ==
			       Element::FlushingState::FLUSHED;
		},
		1000);
	CU_ASSERT_TRUE(gotFlushed);

	stopSessionAndWait(&loop, session, &mediaListener);

	rendererOwner.reset();
	sourceOwner.reset();
}


static void testCxxVideoRendererExtTextureCoverage()
{
	REQUIRE_GL_CONTEXT(glCtx);
	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	struct pdraw_rect renderPos = {0, 0, 64, 48};

	/* Case 1: setupExtTexture with custom DAR where dar <= ar (l.
	 * 1742-1750) */
	struct pdraw_video_renderer_params paramsA = {};
	paramsA.video_texture_dar_width = 1;
	paramsA.video_texture_dar_height = 1;

	IPdraw::IRawVideoSource *sourceA = nullptr;
	ExtLoadVideoTextureListener rendererListenerA;
	IPdraw::IVideoRenderer *rendererA = nullptr;
	setupRendererWithSource(loop,
				session,
				64,
				48,
				renderPos,
				paramsA,
				mediaListener,
				rendererListenerA,
				&sourceA,
				&rendererA);
	auto sourceOwnerA = std::unique_ptr<IPdraw::IRawVideoSource>(sourceA);
	auto rendererOwnerA =
		std::unique_ptr<IPdraw::IVideoRenderer>(rendererA);

	struct mbuf_raw_video_frame_queue *queueA = sourceA->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(queueA);
	struct pdraw_rect contentPos = {};

	/* Push frame to trigger setupExtTexture with dar <= ar (l. 1742-1750)
	 */
	struct mbuf_raw_video_frame *frameA =
		makeRawVideoFrame(vdef_i420, 8, 64, 48, 10, 0);
	int ret = mbuf_raw_video_frame_queue_push(queueA, frameA);
	CU_ASSERT_EQUAL(ret, 0);
	mbuf_raw_video_frame_unref(frameA);

	bool gotA = loop.pumpUntil(
		[&rendererListenerA]() {
			return rendererListenerA.mRenderReadyCount.load() >= 1;
		},
		1000);
	CU_ASSERT_TRUE(gotA);

	ret = rendererA->render(&contentPos);
	CU_ASSERT_EQUAL(ret, 0);

	stopSessionAndWait(&loop, session, &mediaListener);
	rendererOwnerA.reset();
	sourceOwnerA.reset();

	/* Case 2: SAR < 1.0 (l. 1771-1779) and Invalid SAR in
	 * loadExternalVideoFrame (l. 1603-1608) */
	MediaTrackingListener mediaListenerB;
	TestSession testSessionB(&loop, &mediaListenerB);
	IPdraw *sessionB = testSessionB.get();

	struct pdraw_video_renderer_params paramsB = {};
	IPdraw::IRawVideoSource *sourceB = nullptr;
	ExtLoadVideoTextureListener rendererListenerB;
	IPdraw::IVideoRenderer *rendererB = nullptr;
	setupRendererWithSource(loop,
				sessionB,
				64,
				48,
				renderPos,
				paramsB,
				mediaListenerB,
				rendererListenerB,
				&sourceB,
				&rendererB);
	auto sourceOwnerB = std::unique_ptr<IPdraw::IRawVideoSource>(sourceB);
	auto rendererOwnerB =
		std::unique_ptr<IPdraw::IVideoRenderer>(rendererB);

	struct mbuf_raw_video_frame_queue *queueB = sourceB->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(queueB);

	/* Create a frame with SAR < 1.0 (sarWidth=1, sarHeight=2) to trigger l.
	 * 1771-1779 */
	struct mbuf_raw_video_frame *frameSar = makeRawVideoFrame(
		vdef_i420, 8, 64, 48, 10, 0, nullptr, 1000000, 1, 2);
	CU_ASSERT_PTR_NOT_NULL_FATAL(frameSar);

	ret = mbuf_raw_video_frame_queue_push(queueB, frameSar);
	CU_ASSERT_EQUAL(ret, 0);
	mbuf_raw_video_frame_unref(frameSar);

	bool gotB1 = loop.pumpUntil(
		[&rendererListenerB]() {
			return rendererListenerB.mRenderReadyCount.load() >= 1;
		},
		1000);
	CU_ASSERT_TRUE(gotB1);

	ret = rendererB->render(&contentPos);
	CU_ASSERT_EQUAL(ret, 0);

	stopSessionAndWait(&loop, sessionB, &mediaListenerB);
	rendererOwnerB.reset();
	sourceOwnerB.reset();

	/* Case 3: Custom texture_width with sarHeight=0 -> computes
	 * mExtVideoTextureHeight = 0,
	 * triggering setupExtTexture line 1806 else branch (stopExtLoad l.
	 * 1807-1816) */
	MediaTrackingListener mediaListenerC;
	TestSession testSessionC(&loop, &mediaListenerC);
	IPdraw *sessionC = testSessionC.get();

	struct pdraw_video_renderer_params paramsC = {};
	paramsC.video_texture_width = 64; /* Custom texture width */

	IPdraw::IRawVideoSource *sourceC = nullptr;
	ExtLoadVideoTextureListener rendererListenerC;
	IPdraw::IVideoRenderer *rendererC = nullptr;
	setupRendererWithSource(loop,
				sessionC,
				64,
				48,
				renderPos,
				paramsC,
				mediaListenerC,
				rendererListenerC,
				&sourceC,
				&rendererC);
	auto sourceOwnerC = std::unique_ptr<IPdraw::IRawVideoSource>(sourceC);
	auto rendererOwnerC =
		std::unique_ptr<IPdraw::IVideoRenderer>(rendererC);

	struct mbuf_raw_video_frame_queue *queueC = sourceC->getQueue();
	CU_ASSERT_PTR_NOT_NULL_FATAL(queueC);

	/* Push first frame with sarWidth=1, sarHeight=0 ->
	 * mExtVideoTextureHeight = 0 */
	struct mbuf_raw_video_frame *frameZeroHeight = makeRawVideoFrame(
		vdef_i420, 8, 64, 48, 10, 0, nullptr, 1000000, 1, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(frameZeroHeight);

	ret = mbuf_raw_video_frame_queue_push(queueC, frameZeroHeight);
	CU_ASSERT_EQUAL(ret, 0);
	mbuf_raw_video_frame_unref(frameZeroHeight);

	bool gotC = loop.pumpUntil(
		[&rendererListenerC]() {
			return rendererListenerC.mRenderReadyCount.load() >= 1;
		},
		1000);
	CU_ASSERT_TRUE(gotC);

	/* First frame render triggers setupExtTexture ->
	 * mExtVideoTextureHeight=0 -> stopExtLoad (l. 1806-1816) */
	ret = rendererC->render(&contentPos);

	stopSessionAndWait(&loop, sessionC, &mediaListenerC);
	rendererOwnerC.reset();
	sourceOwnerC.reset();
}


static void testCxxVideoRendererWrapperGuardsAfterElementCleared()
{
	static constexpr uint32_t kWidth = 64;
	static constexpr uint32_t kHeight = 64;

	TestPompLoop loop;
	MediaTrackingListener mediaListener;
	TestSession testSession(&loop, &mediaListener);
	IPdraw *session = testSession.get();

	/* Must be current before createVideoRenderer(): its constructor
	 * calls GlVideoRenderer::setup() synchronously, which queries
	 * GL_FRAMEBUFFER_BINDING. */
	REQUIRE_GL_CONTEXT(glContext);

	struct pdraw_rect renderPos = {0, 0, kWidth, kHeight};
	struct pdraw_video_renderer_params rendererParams = {};
	VideoRendererTrackingListener rendererListener;
	IPdraw::IRawVideoSource *source = nullptr;
	IPdraw::IVideoRenderer *renderer = nullptr;
	setupRendererWithSource(loop,
				session,
				kWidth,
				kHeight,
				renderPos,
				rendererParams,
				mediaListener,
				rendererListener,
				&source,
				&renderer);
	auto rendererOwner = std::unique_ptr<IPdraw::IVideoRenderer>(renderer);
	auto sourceOwner = std::unique_ptr<IPdraw::IRawVideoSource>(source);

	stopSessionAndWait(&loop, session, &mediaListener);

	CU_ASSERT_PTR_NULL(
		static_cast<VideoRendererWrapper *>(renderer)->getElement());

	struct pdraw_rect rect = {};
	struct pdraw_video_renderer_params params = {};
	CU_ASSERT_EQUAL(renderer->render(nullptr), -EPROTO);
	CU_ASSERT_EQUAL(renderer->resize(&rect), -EPROTO);
	CU_ASSERT_EQUAL(renderer->setMediaId(0), -EPROTO);
	CU_ASSERT_EQUAL(renderer->getMediaId(), (unsigned int)-EPROTO);
	CU_ASSERT_EQUAL(renderer->setParams(&params), -EPROTO);
	CU_ASSERT_EQUAL(renderer->getParams(&params), -EPROTO);

	/* renderer's destructor issues real GL calls: must run while
	 * glContext is still bound. */
	rendererOwner.reset();
	sourceOwner.reset();
}


#endif /* PDRAW_TEST_RENDERER_VIDEO_GL_ENABLED */


CU_TestInfo g_pdraw_test_pipeline_renderer_video[] = {
#ifdef PDRAW_TEST_RENDERER_VIDEO_GL_ENABLED
	{FN("testCxxVideoRendererRendersRealFrame"),
	 testCxxVideoRendererRendersRealFrame},
	{FN("testCxxVideoRendererRendersRealFrameNV12"),
	 testCxxVideoRendererRendersRealFrameNV12},
	{FN("testCxxVideoRendererRendersRealFrameGray"),
	 testCxxVideoRendererRendersRealFrameGray},
	{FN("testCxxVideoRendererRendersRealFrameRaw16"),
	 testCxxVideoRendererRendersRealFrameRaw16},
	{FN("testCxxVideoRendererRendersRealFrameI420_10_16LE"),
	 testCxxVideoRendererRendersRealFrameI420_10_16LE},
	{FN("testCxxVideoRendererRendersRealFrameNV12_10_16LE_HIGH"),
	 testCxxVideoRendererRendersRealFrameNV12_10_16LE_HIGH},
	{FN("testCxxVideoRendererRendersRealFrameRaw32"),
	 testCxxVideoRendererRendersRealFrameRaw32},
	{FN("testCxxVideoRendererHistogramComputed"),
	 testCxxVideoRendererHistogramComputed},
	{FN("testCxxVideoRendererDrainFiresCallback"),
	 testCxxVideoRendererDrainFiresCallback},
	{FN("testCxxVideoRendererFlushFiresCallbackAndFreezesFrame"),
	 testCxxVideoRendererFlushFiresCallbackAndFreezesFrame},
	{FN("testCxxVideoRendererExtTextureWithDAR"),
	 testCxxVideoRendererExtTextureWithDAR},
	{FN("testCxxVideoRendererAdaptiveSchedulingRendersFrames"),
	 testCxxVideoRendererAdaptiveSchedulingRendersFrames},
	{FN("testCxxVideoRendererResizeUpdatesContentPos"),
	 testCxxVideoRendererResizeUpdatesContentPos},
	{FN("testCxxVideoRendererSetParamsGetParamsRoundtrip"),
	 testCxxVideoRendererSetParamsGetParamsRoundtrip},
	{FN("testCxxVideoRendererMediaRemovedWhenSourceDestroyed"),
	 testCxxVideoRendererMediaRemovedWhenSourceDestroyed},
	{FN("testCxxVideoRendererSwitchMediaId"),
	 testCxxVideoRendererSwitchMediaId},
	{FN("testCxxVideoRendererBlurAndTransitions"),
	 testCxxVideoRendererBlurAndTransitions},
	{FN("testCxxVideoRendererFillModePadBlurExtend"),
	 testCxxVideoRendererFillModePadBlurExtend},
	{FN("testCxxVideoRendererFillModePadBlurExtendMirrored"),
	 testCxxVideoRendererFillModePadBlurExtendMirrored},
	{FN("testCxxVideoRendererFillModePadBlurCropWide"),
	 testCxxVideoRendererFillModePadBlurCropWide},
	{FN("testCxxVideoRendererOverexposureZebrasEnabled"),
	 testCxxVideoRendererOverexposureZebrasEnabled},
	{FN("testCxxVideoRendererExtTexturePadHistogramBlur"),
	 testCxxVideoRendererExtTexturePadHistogramBlur},
	{FN("testCxxVideoRendererNv21PadHistogramBlur"),
	 testCxxVideoRendererNv21PadHistogramBlur},
	{FN("testCxxVideoRendererResolutionChangeSetsRestartFlag"),
	 testCxxVideoRendererResolutionChangeSetsRestartFlag},
	{FN("testCxxVideoRendererFramerateChangeSetsRestartFlag"),
	 testCxxVideoRendererFramerateChangeSetsRestartFlag},
	{FN("testCxxVideoRendererSessionMetaUpdate"),
	 testCxxVideoRendererSessionMetaUpdate},
	{FN("testCxxVideoRendererExtTextureDarNoWidth"),
	 testCxxVideoRendererExtTextureDarNoWidth},
	{FN("testCxxVideoRendererExtTextureWidthNoDar"),
	 testCxxVideoRendererExtTextureWidthNoDar},
	{FN("testCxxVideoRendererExtTextureNoWidthNoDar"),
	 testCxxVideoRendererExtTextureNoWidthNoDar},
	{FN("testCxxVideoRendererAutoNormalizationThermal"),
	 testCxxVideoRendererAutoNormalizationThermal},
	{FN("testCxxVideoRendererAutoNormalizationThermalNoMetadataResets"),
	 testCxxVideoRendererAutoNormalizationThermalNoMetadataResets},
	{FN("testCxxVideoRendererMbStatusOverlayFromDbgFlagsEnv"),
	 testCxxVideoRendererMbStatusOverlayFromDbgFlagsEnv},
	{FN("testCxxVideoRendererMalformedDbgFlagsEnvIgnored"),
	 testCxxVideoRendererMalformedDbgFlagsEnvIgnored},
	{FN("testCxxVideoRendererVideoPresStatsTimerFires"),
	 testCxxVideoRendererVideoPresStatsTimerFires},
	{FN("testCVideoRendererListenerCallbacks"),
	 testCVideoRendererListenerCallbacks},
	{FN("testCxxVideoRendererSchedulingModes"),
	 testCxxVideoRendererSchedulingModes},
	{FN("testCxxVideoRendererSimplifiedRendering"),
	 testCxxVideoRendererSimplifiedRendering},
	{FN("testCxxVideoRendererFillModeFitAndCropContentPos"),
	 testCxxVideoRendererFillModeFitAndCropContentPos},
	{FN("testCxxVideoRendererEnableVerticalMirror"),
	 testCxxVideoRendererEnableVerticalMirror},
	{FN("testCxxVideoRendererResizeInvalidArgs"),
	 testCxxVideoRendererResizeInvalidArgs},
	{FN("testCxxVideoRendererCustomViewAndProjMatrix"),
	 testCxxVideoRendererCustomViewAndProjMatrix},
	{FN("testCxxVideoRendererAdaptiveSchedulingFullCoverage"),
	 testCxxVideoRendererAdaptiveSchedulingFullCoverage},
	{FN("testCxxVideoRendererAsyncDrainOnFlush"),
	 testCxxVideoRendererAsyncDrainOnFlush},
	{FN("testCxxVideoRendererExtTextureCoverage"),
	 testCxxVideoRendererExtTextureCoverage},
	{FN("testCxxVideoRendererWrapperGuardsAfterElementCleared"),
	 testCxxVideoRendererWrapperGuardsAfterElementCleared},
#endif
	CU_TEST_INFO_NULL,
};
