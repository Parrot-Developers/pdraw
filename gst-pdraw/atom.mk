LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := gst-pdraw
LOCAL_CATEGORY_PATH := multimedia/gstreamer
LOCAL_DESCRIPTION := GStreamer PDrAW plugin
LOCAL_DESTDIR := usr/lib/gstreamer-1.0
LOCAL_MODULE_FILENAME := libgstpdraw.so
LOCAL_CODECHECK_C := none

LOCAL_LIBRARIES := \
	glib \
	gstreamer \
	gst-plugins-base \
	libmedia-buffers \
	libmedia-buffers-memory-generic \
	libpdraw \
	libpdraw-backend \
	libulog \
	libvideo-defs

LOCAL_SRC_FILES := \
	gstpdrawsrc.c

include $(BUILD_SHARED_LIBRARY)
