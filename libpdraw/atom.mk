
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := libpdraw
LOCAL_DESCRIPTION := Parrot Drones Audio and Video Vector library
LOCAL_CATEGORY_PATH := libs

LOCAL_CONFIG_FILES := pdraw.in
$(call load-config)

LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/include
# Public API headers - top level headers first
# This header list is currently used to generate a python binding
LOCAL_EXPORT_CUSTOM_VARIABLES := LIBPDRAW_HEADERS=$\
	$(LOCAL_PATH)/include/pdraw/pdraw.h:$\
	$(LOCAL_PATH)/include/pdraw/pdraw_defs.h;
LOCAL_CFLAGS := -DPDRAW_API_EXPORTS -fvisibility=hidden -D_USE_MATH_DEFINES -D_GNU_SOURCE
LOCAL_CXXFLAGS := -std=c++17
LOCAL_EXPORT_CXXFLAGS := -std=c++17

LIBPDRAW_SRC_FILES := \
	$(call all-cpp-files-under,src)

LOCAL_SRC_FILES := $(LIBPDRAW_SRC_FILES)

LIBPDRAW_LIBRARIES := \
	eigen \
	libaac \
	libaudio-decode \
	libaudio-defs \
	libaudio-encode \
	libaudio-encode-core \
	libfutils \
	libh264 \
	libh265 \
	libmedia-buffers \
	libmedia-buffers-cpp \
	libmedia-buffers-memory \
	libmedia-buffers-memory-generic \
	libmp4 \
	libphoto-metadata-defs \
	libpomp \
	librtp \
	librtsp \
	libsdp \
	libtransport-packet \
	libtransport-socket \
	libulog \
	libvideo-decode \
	libvideo-defs \
	libvideo-encode \
	libvideo-encode-core \
	libvideo-metadata \
	libvideo-scale \
	libvideo-scale-core \
	libvideo-streaming

# LOCAL_LIBRARIES set below

LIBPDRAW_CONDITIONAL_LIBRARIES := \
	OPTIONAL:json \
	OPTIONAL:libdng-parrot \
	OPTIONAL:libjfif \
	OPTIONAL:libmux \
	OPTIONAL:librtmp \
	OPTIONAL:libvideo-ipc \
	OPTIONAL:libvideo-ipc-client-config \
	CONFIG_PDRAW_VIPC_BACKEND_DMABUF:libvideo-ipc-dmabuf-be \
	CONFIG_PDRAW_VIPC_BACKEND_DMABUF:libmedia-buffers-memory-ion \
	CONFIG_PDRAW_VIPC_BACKEND_DMABUF:libmedia-buffers-memory-vacq \
	CONFIG_PDRAW_VIPC_BACKEND_HISI:libvideo-ipc-hisibe \
	CONFIG_PDRAW_VIPC_BACKEND_HISI:libmedia-buffers-memory-hisi \
	CONFIG_PDRAW_VIPC_BACKEND_NETWORK_CBUF:libvideo-ipc-network-cbuf-be \
	CONFIG_PDRAW_VIPC_BACKEND_NETWORK_HISI:libvideo-ipc-network-hisi-be \
	CONFIG_PDRAW_VIPC_BACKEND_SHM:libvideo-ipc-shmbe \
	CONFIG_PDRAW_USE_ALSA:alsa-lib

# LOCAL_CONDITIONAL_LIBRARIES set below

ifeq ("$(TARGET_OS)","windows")
  LIBPDRAW_CFLAGS += -D_WIN32_WINNT=0x0600
  LIBPDRAW_LDLIBS += -lws2_32
endif

ifdef CONFIG_PDRAW_USE_GL
  ifeq ($(TARGET_CPU),$(filter %$(TARGET_CPU),s905d3 s905x3))
    LIBPDRAW_LDLIBS += -lGLESv2
    LIBPDRAW_CONDITIONAL_LIBRARIES += \
	CONFIG_PDRAW_USE_GL:am-gpu
  else ifeq ($(TARGET_CPU),qcs405)
    LIBPDRAW_CFLAGS += -DUSE_GLES2
    LIBPDRAW_LIBRARIES += \
	glesv2 \
	egl
  else ifeq ("$(TARGET_OS)-$(TARGET_OS_FLAVOUR)","linux-native")
    LIBPDRAW_CONDITIONAL_LIBRARIES += \
	CONFIG_PDRAW_USE_GL:opengl
  else ifeq ("$(TARGET_OS)-$(TARGET_OS_FLAVOUR)","linux-android")
    LIBPDRAW_LDLIBS += -lEGL -lGLESv2 -landroid
  else ifeq ("$(TARGET_OS)-$(TARGET_OS_FLAVOUR)","darwin-native")
    LIBPDRAW_CFLAGS += -DGL_SILENCE_DEPRECATION
    LIBPDRAW_LDLIBS += -framework OpenGL
  else ifeq ($(TARGET_OS_FLAVOUR),$(filter %$(TARGET_OS_FLAVOUR),iphoneos iphonesimulator))
    LIBPDRAW_CFLAGS += -DGLES_SILENCE_DEPRECATION
    LIBPDRAW_LDLIBS += -framework OpenGLES
  else ifeq ("$(TARGET_OS)","windows")
    LIBPDRAW_CFLAGS += -DEPOXY_SHARED
    LIBPDRAW_LDLIBS += -lepoxy
  endif
endif

LOCAL_CFLAGS += $(LIBPDRAW_CFLAGS)
LOCAL_LDLIBS += $(LIBPDRAW_LDLIBS)
LOCAL_LIBRARIES := $(LIBPDRAW_LIBRARIES)
LOCAL_CONDITIONAL_LIBRARIES := $(LIBPDRAW_CONDITIONAL_LIBRARIES)

include $(BUILD_LIBRARY)

###############################################################################
# libpdraw unit tests (CUnit)
###############################################################################

ifdef TARGET_TEST

include $(CLEAR_VARS)

LOCAL_MODULE := tst-libpdraw
LOCAL_CATEGORY_PATH := multimedia/tests
LOCAL_DESCRIPTION := libpdraw CUnit unit tests

# Reuse libpdraw's autoconf header directly instead of generating a separate
# autoconf-tst-libpdraw.h: load-config always names the output after
# LOCAL_MODULE, so calling it here would produce a distinct (and redundant)
# header. Expressing the dependency via LOCAL_DEPENDS_MODULES ensures
# autoconf-libpdraw.h exists before this module compiles.
LOCAL_DEPENDS_MODULES += libpdraw

# LIBPDRAW_CFLAGS/LIBPDRAW_LDLIBS carry the platform/GL-specific additions
# from the ifeq/ifdef blocks above (Windows, CONFIG_PDRAW_USE_GL): those
# blocks only touch LOCAL_CFLAGS/LOCAL_LDLIBS directly for the libpdraw
# module itself, so without this they are silently missing here too (same
# root cause as LIBPDRAW_LIBRARIES/LIBPDRAW_CONDITIONAL_LIBRARIES below,
# just for flags/link libs instead of module names).
#
# BUILD_LIBMUX is added here for the same reason as BUILD_LIBRTMP: libmux is
# filtered out of LOCAL_CONDITIONAL_LIBRARIES below (no real libmux linked
# into the test binary), so Alchemy never auto-defines it. The pdraw_*_mux.cpp
# production files and their CUnit suites (pipeline_demuxer_stream_mux,
# pipeline_muxer_stream_rtsp_mux) are entirely guarded by #ifdef BUILD_LIBMUX;
# tests/mock_libmux.{hpp,cpp} provides the handful of mux_*/mux_ip_proxy_*
# symbols they need, so no real libmux dependency is introduced.
LOCAL_CFLAGS := -DPDRAW_API_EXPORTS -D_GNU_SOURCE -D_USE_MATH_DEFINES \
	$(LIBPDRAW_CFLAGS) \
	-DBUILD_LIBRTMP \
	-DBUILD_LIBMUX \
	-include $(TARGET_OUT_BUILD)/libpdraw/autoconf-libpdraw.h

# CONFIG_VDEC_TURBOJPEG belongs to libvideo-decode's own Kconfig namespace
# (packages/libvideo-decode/config.in), not libpdraw's, so it is absent from
# autoconf-libpdraw.h above. Tests that exercise the JPEG decode path (e.g.
# VideoDecoder::start() on a JPEG media) need to know at compile time whether
# the TurboJPEG implementation was actually linked into libvideo-decode, so
# surface it here explicitly for the test build only.
ifdef CONFIG_VDEC_TURBOJPEG
  LOCAL_CFLAGS += -DVDEC_TURBOJPEG
endif

LOCAL_LDLIBS := $(LIBPDRAW_LDLIBS)
LOCAL_CXXFLAGS := -std=c++17
LOCAL_C_INCLUDES := $(LOCAL_PATH)/src $(LOCAL_PATH)/include

LOCAL_SRC_FILES := \
	$(LIBPDRAW_SRC_FILES) \
	$(call all-cpp-files-under,tests) \
	$(call all-c-files-under,tests)

# libvideo-raw: only needed by tests/test_pipeline_sourcesink_raw.cpp (raw
#               YUV file reader/writer).
# libaudio-raw: only needed by tests/test_pipeline_sourcesink_audio.cpp
#               (WAV file reader/writer). Neither is used by the production
#               libpdraw sources (libaac already is, via LIBPDRAW_LIBRARIES).
LOCAL_LIBRARIES := \
	$(LIBPDRAW_LIBRARIES) \
	libcunit \
	libvideo-raw \
	libaudio-raw

LOCAL_CONDITIONAL_LIBRARIES := \
	$(filter-out librtmp libmux,$(LIBPDRAW_CONDITIONAL_LIBRARIES))

# tests/test_pipeline_renderer_video.cpp creates its own offscreen EGL
# context (libpdraw's own GL renderer code never does -- confirmed by
# reading pdraw_renderer_video_gl.cpp/pdraw_gl_video.cpp: neither calls any
# EGL/GLX function itself, both assume the application already made a
# context current). Needed only here, not by LIBPDRAW_CONDITIONAL_LIBRARIES
# above (production libpdraw has no reason to link EGL itself). Scoped to
# linux-native for now, matching libpdraw-streamsharing's atom.mk (the only
# other place in the monorepo creating an offscreen GL context for pdraw
# rendering) and this test file's own
# "#if defined(__linux__) && !defined(__ANDROID__)" guard.
ifdef CONFIG_PDRAW_USE_GL
  ifeq ("$(TARGET_OS)-$(TARGET_OS_FLAVOUR)","linux-native")
    LOCAL_LIBRARIES += egl
  endif
endif

include $(BUILD_EXECUTABLE)

endif # TARGET_TEST
