
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := libpdraw-vsink
LOCAL_CATEGORY_PATH := libs
LOCAL_DESCRIPTION := PDrAW video sink wrapper library
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/include
LOCAL_CFLAGS := -DPDRAW_VSINK_API_EXPORTS -fvisibility=hidden -std=gnu99 -D_GNU_SOURCE
LOCAL_SRC_FILES := \
	src/pdraw_vsink.c
LOCAL_LIBRARIES := \
	libfutils \
	libmedia-buffers \
	libmedia-buffers-memory \
	libmedia-buffers-memory-generic \
	libpdraw \
	libpomp \
	libulog

include $(BUILD_LIBRARY)


include $(CLEAR_VARS)

LOCAL_MODULE := pdraw-vsink-test
LOCAL_DESCRIPTION := PDrAW video sink wrapper library test program
LOCAL_CATEGORY_PATH := multimedia
LOCAL_SRC_FILES := tests/pdraw_vsink_test.c
LOCAL_LIBRARIES := \
	libmedia-buffers \
	libpdraw-vsink \
	libulog \
	libvideo-defs \
	libvideo-metadata

include $(BUILD_EXECUTABLE)
