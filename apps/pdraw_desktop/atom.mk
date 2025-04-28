
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := pdraw
LOCAL_DESCRIPTION := PDrAW desktop player application
LOCAL_CATEGORY_PATH := multimedia
LOCAL_CFLAGS := -D_USE_MATH_DEFINES -D_GNU_SOURCE
LOCAL_SRC_FILES := \
	pdraw_desktop.c \
	pdraw_desktop_ext_tex.c \
	pdraw_desktop_ui.c \
	pdraw_desktop_view.cpp
LOCAL_LIBRARIES := \
	eigen \
	libfutils \
	libmedia-buffers \
	libpdraw \
	libpdraw-backend \
	libpdraw-gles2hud \
	libpomp \
	libulog \
	libvideo-defs \
	libvideo-metadata \
	sdl2

LOCAL_CONDITIONAL_LIBRARIES := \
	OPTIONAL:libpdraw-overlayer

ifeq ("$(TARGET_OS)-$(TARGET_OS_FLAVOUR)","linux-native")
  LOCAL_LIBRARIES += \
	opengl
else ifeq ("$(TARGET_OS)-$(TARGET_OS_FLAVOUR)","darwin-native")
  LOCAL_LDLIBS += \
	-framework OpenGL
else ifeq ("$(TARGET_OS)","windows")
  LOCAL_CFLAGS += -D_WIN32_WINNT=0x0600 -DEPOXY_SHARED
  LOCAL_LDLIBS += -lws2_32 -lepoxy
endif

include $(BUILD_EXECUTABLE)
