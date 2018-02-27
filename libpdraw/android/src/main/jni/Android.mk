LOCAL_PATH := $(call my-dir)

# JNI Wrapper
include $(CLEAR_VARS)

LOCAL_MODULE := libpdraw_android
LOCAL_SRC_FILES := pdraw_jni.c pdraw_dummy.cpp
LOCAL_LDLIBS := -llog -landroid -lEGL
LOCAL_SHARED_LIBRARIES := libpdraw

include $(BUILD_SHARED_LIBRARY)
