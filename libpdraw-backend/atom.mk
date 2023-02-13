
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := libpdraw-backend
LOCAL_CATEGORY_PATH := libs
LOCAL_DESCRIPTION := Parrot Drones Awesome Video Viewer back-end library
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
LOCAL_DESCRIPTION := Parrot Drones Awesome Video Viewer back-end library test program
LOCAL_SRC_FILES := \
	tests/pdraw_backend_test.c
LOCAL_LIBRARIES := \
	libpdraw \
	libpdraw-backend \
	libulog

include $(BUILD_EXECUTABLE)
