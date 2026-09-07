
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := libpdraw-vsink
LOCAL_CATEGORY_PATH := libs
LOCAL_DESCRIPTION := PDrAW video sink wrapper library
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/include
# LOCAL_CFLAGS/LOCAL_CXXFLAGS are separate flag sets in Alchemy (one per
# source language, like Android's ndk-build) -- since every source file
# here is now .cpp, -DPDRAW_VSINK_API_EXPORTS/-fvisibility=hidden/
# -D_GNU_SOURCE have to be repeated in LOCAL_CXXFLAGS too, or the exported
# C-ABI symbols in pdraw_vsink_wrapper.cpp would silently keep hidden
# visibility (and pthread_setname_np's declaration in pdraw_vsink.cpp's
# loopThread() would disappear without _GNU_SOURCE).
LOCAL_CFLAGS := -DPDRAW_VSINK_API_EXPORTS -fvisibility=hidden -D_GNU_SOURCE
LOCAL_CXXFLAGS := \
	-DPDRAW_VSINK_API_EXPORTS -fvisibility=hidden -D_GNU_SOURCE -std=c++17
LOCAL_EXPORT_CXXFLAGS := -std=c++17
LOCAL_SRC_FILES := \
	src/pdraw_vsink.cpp \
	src/pdraw_vsink_coded.cpp \
	src/pdraw_vsink_raw.cpp \
	src/pdraw_vsink_wrapper.cpp
LOCAL_LIBRARIES := \
	libfutils \
	libmedia-buffers \
	libmedia-buffers-memory \
	libmedia-buffers-memory-generic \
	libpdraw \
	libpomp \
	libulog \
	libvideo-metadata

include $(BUILD_LIBRARY)


include $(CLEAR_VARS)

LOCAL_MODULE := pdraw-vsink-test
LOCAL_DESCRIPTION := PDrAW video sink wrapper library test program
LOCAL_CATEGORY_PATH := multimedia
LOCAL_SRC_FILES := tools/pdraw_vsink_test.c
LOCAL_LIBRARIES := \
	libmedia-buffers \
	libpdraw \
	libpdraw-vsink \
	libulog \
	libvideo-defs \
	libvideo-metadata

include $(BUILD_EXECUTABLE)


include $(CLEAR_VARS)

LOCAL_MODULE := pdraw-vsink-test-cpp
LOCAL_DESCRIPTION := PDrAW video sink wrapper library test program (C++)
LOCAL_CATEGORY_PATH := multimedia
LOCAL_CXXFLAGS := -std=c++17
LOCAL_SRC_FILES := tools/pdraw_vsink_test.cpp
LOCAL_LIBRARIES := \
	libmedia-buffers \
	libpdraw \
	libpdraw-vsink \
	libulog \
	libvideo-defs \
	libvideo-metadata

include $(BUILD_EXECUTABLE)


include $(CLEAR_VARS)

LOCAL_MODULE := pdraw-vsink-test-opencv
LOCAL_DESCRIPTION := PDrAW video sink wrapper library face detection example (OpenCV)
LOCAL_CATEGORY_PATH := multimedia
LOCAL_CXXFLAGS := -std=c++17
LOCAL_SRC_FILES := tools/pdraw_vsink_test_opencv.cpp
LOCAL_LIBRARIES := \
	libmedia-buffers \
	libpdraw \
	libpdraw-vsink \
	libulog \
	libvideo-defs \
	libvideo-metadata
# Prefer the system OpenCV (pkg-config) if installed, else fall back to the
# in-tree opencv2 source build (needs config.in's select block). Only the
# opencv2 case is staged (out/.../staging) by the Alchemy build -- the
# system OpenCV's data only ever exists at its real host path, so the
# PRODUCT_ROOT_CFG prefix at runtime must not apply to it (see
# defaultCascadePath() in the .cpp).
pdraw_vsink_cascade_dir := /usr/share/OpenCV/haarcascades
pdraw_vsink_cascade_staged := 0
ifneq ("$(call is-module-in-build-config,opencv)","")
LOCAL_LIBRARIES += opencv
ifeq ("$(shell pkg-config --exists opencv4; echo $$?)","0")
pdraw_vsink_cascade_dir := /usr/share/opencv4/haarcascades
endif
else
LOCAL_LIBRARIES += opencv2
LOCAL_CONFIG_FILES := config.in
pdraw_vsink_cascade_staged := 1
endif
LOCAL_CXXFLAGS += \
	-DPDRAW_VSINK_DEFAULT_CASCADE_PATH=\"$(pdraw_vsink_cascade_dir)/haarcascade_frontalface_default.xml\" \
	-DPDRAW_VSINK_CASCADE_PATH_STAGED=$(pdraw_vsink_cascade_staged)

include $(BUILD_EXECUTABLE)


ifdef TARGET_TEST

include $(CLEAR_VARS)

LOCAL_MODULE := tst-libpdraw-vsink
LOCAL_CATEGORY_PATH := multimedia/tests
LOCAL_DESCRIPTION := libpdraw-vsink CUnit unit tests
# LOCAL_C_INCLUDES: src/ so tests can white-box-include pdraw_vsink_priv.hpp
# (the Vsink class's members, the raw/coded get_frame internals -- matches
# how libpdraw-streamsharing's own tests do it); include/ so the recompiled
# src/pdraw_vsink*.cpp files (and the tests themselves) can resolve the
# public <pdraw-vsink/pdraw_vsink.h>/.hpp headers -- normally reached via
# LOCAL_LIBRARIES := libpdraw-vsink's LOCAL_EXPORT_C_INCLUDES, but this
# module recompiles the sources directly instead of linking that library,
# so it isn't a dependency here and its export path isn't inherited.
LOCAL_C_INCLUDES := $(LOCAL_PATH)/src $(LOCAL_PATH)/include
LOCAL_CXXFLAGS := -D_GNU_SOURCE -std=c++17
LOCAL_SRC_FILES := \
	src/pdraw_vsink.cpp \
	src/pdraw_vsink_coded.cpp \
	src/pdraw_vsink_raw.cpp \
	src/pdraw_vsink_wrapper.cpp \
	tests/test_rtsp_server.cpp \
	tests/test_util_real.cpp \
	tests/test_util_start.cpp \
	tests/test_start.cpp \
	tests/test_media_dispatch.cpp \
	tests/test_get_frame_raw.cpp \
	tests/test_get_frame_coded.cpp \
	tests/test_frame_ready_cb.cpp \
	tests/test_flush_drain.cpp \
	tests/test_stop.cpp \
	tests/test_main.cpp
# Real libpdraw is linked deliberately, unlike a typical dependency mock:
# libpdraw already has its own CUnit suite (tst-libpdraw), so these tests
# trust it and only validate libpdraw-vsink's own logic (state machine,
# threading/condvar handshake, media-type dispatch, get_frame semantics,
# teardown), driven through a real local pdraw pipeline. librtsp/libsdp are
# test-only additions (tests/test_rtsp_server.cpp), not used by production
# libpdraw-vsink.
LOCAL_LIBRARIES := \
	libcunit \
	libfutils \
	libmedia-buffers \
	libmedia-buffers-memory \
	libmedia-buffers-memory-generic \
	libpdraw \
	libpomp \
	librtsp \
	libsdp \
	libulog \
	libvideo-defs \
	libvideo-metadata

include $(BUILD_EXECUTABLE)

endif # TARGET_TEST
