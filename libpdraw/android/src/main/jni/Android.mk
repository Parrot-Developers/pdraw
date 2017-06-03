LOCAL_PATH := $(call my-dir)

# JNI Wrapper
include $(CLEAR_VARS)

LOCAL_MODULE := libpdraw_android
LOCAL_SRC_FILES := pdraw_jni.c
LOCAL_LDLIBS := -llog -landroid
LOCAL_SHARED_LIBRARIES := libpdraw libulog libpomp libvideo-metadata libARStream2 libmp4 libh264 json

include $(BUILD_SHARED_LIBRARY)
