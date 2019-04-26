
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := libpdraw-vsink
LOCAL_CATEGORY_PATH := libs
LOCAL_DESCRIPTION := Parrot Drones Awesome Video Viewer video sink wrapper library
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/include
LOCAL_CFLAGS := -DPDRAW_VSINK_API_EXPORTS -fvisibility=hidden -std=gnu99
LOCAL_SRC_FILES := \
	src/pdraw_vsink.c
LOCAL_LIBRARIES := \
	libfutils \
	libpdraw \
	libpomp \
	libulog \
	libvideo-buffers \
	libvideo-buffers-generic \
	libvideo-metadata

include $(BUILD_LIBRARY)


include $(CLEAR_VARS)

LOCAL_MODULE := pdraw_vsink_test
LOCAL_DESCRIPTION := Parrot Drones Awesome Video Viewer video sink wrapper library test program
LOCAL_CATEGORY_PATH := multimedia
LOCAL_SRC_FILES := tests/pdraw_vsink_test.c
LOCAL_LIBRARIES := \
	libpdraw-vsink \
	libulog \
	libvideo-buffers \
	libvideo-buffers-generic

include $(BUILD_EXECUTABLE)
