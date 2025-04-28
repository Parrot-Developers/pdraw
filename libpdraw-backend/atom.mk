
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := libpdraw-backend
LOCAL_CATEGORY_PATH := libs
LOCAL_DESCRIPTION := PDrAW back-end library
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/include
LOCAL_CFLAGS := -DPDRAW_BACKEND_API_EXPORTS -fvisibility=hidden -std=gnu99
LOCAL_SRC_FILES := \
	src/pdraw_backend_impl.cpp \
	src/pdraw_backend_wrapper.cpp
LOCAL_LIBRARIES := \
	libfutils \
	libpdraw \
	libpomp \
	libulog

include $(BUILD_LIBRARY)


include $(CLEAR_VARS)

LOCAL_MODULE := pdraw-backend-test
LOCAL_CATEGORY_PATH := multimedia
LOCAL_DESCRIPTION := PDrAW back-end library test program
LOCAL_SRC_FILES := \
	tests/pdraw_backend_test.c
LOCAL_LIBRARIES := \
	libpdraw \
	libpdraw-backend \
	libulog

include $(BUILD_EXECUTABLE)


include $(CLEAR_VARS)

LOCAL_MODULE := pdraw-vipcsourcesink-test
LOCAL_CATEGORY_PATH := multimedia
LOCAL_DESCRIPTION := PDrAW video IPC source to sink test program
LOCAL_SRC_FILES := \
	tests/pdraw_vipcsourcesink_test.c
LOCAL_LIBRARIES := \
	libmedia-buffers \
	libmedia-buffers-memory \
	libmedia-buffers-memory-generic \
	libpdraw \
	libpdraw-backend \
	libulog \
	libvideo-defs \
	libvideo-raw

include $(BUILD_EXECUTABLE)


include $(CLEAR_VARS)

LOCAL_MODULE := pdraw-rawsourcesink-test
LOCAL_CATEGORY_PATH := multimedia
LOCAL_DESCRIPTION := PDrAW raw video source to sink test program
LOCAL_SRC_FILES := \
	tests/pdraw_rawsourcesink_test.c
LOCAL_LIBRARIES := \
	libmedia-buffers \
	libmedia-buffers-memory \
	libmedia-buffers-memory-generic \
	libpdraw \
	libpdraw-backend \
	libulog \
	libvideo-defs \
	libvideo-raw

include $(BUILD_EXECUTABLE)


include $(CLEAR_VARS)

LOCAL_MODULE := pdraw-codedsourcesink-test
LOCAL_CATEGORY_PATH := multimedia
LOCAL_DESCRIPTION := PDrAW coded video source to sink test program
LOCAL_SRC_FILES := \
	tests/pdraw_codedsourcesink_test.c
LOCAL_LIBRARIES := \
	libh264 \
	libh265 \
	libmedia-buffers \
	libmedia-buffers-memory \
	libmedia-buffers-memory-generic \
	libpdraw \
	libpdraw-backend \
	libulog \
	libvideo-defs

ifeq ("$(TARGET_OS)","windows")
  LOCAL_LDLIBS += -lws2_32
endif

include $(BUILD_EXECUTABLE)


include $(CLEAR_VARS)

LOCAL_MODULE := pdraw-audiosourcesink-test
LOCAL_CATEGORY_PATH := multimedia
LOCAL_DESCRIPTION := PDrAW audio source to sink test program
LOCAL_SRC_FILES := \
	tests/pdraw_audiosourcesink_test.c
LOCAL_LIBRARIES := \
	libaudio-defs \
	libaudio-raw \
	libmedia-buffers \
	libmedia-buffers-memory \
	libmedia-buffers-memory-generic \
	libpdraw \
	libpdraw-backend \
	libulog

include $(BUILD_EXECUTABLE)
