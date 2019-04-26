
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := pdraw
LOCAL_DESCRIPTION := Parrot Drones Awesome Video Viewer desktop application
LOCAL_CATEGORY_PATH := multimedia
LOCAL_SRC_FILES := \
	pdraw_desktop.c \
	pdraw_desktop_ext_tex.c \
	pdraw_desktop_ui.c \
	pdraw_desktop_view.cpp
LOCAL_LIBRARIES := \
	eigen \
	libpdraw \
	libpdraw-backend \
	libpdraw-gles2hud \
	libpomp \
	libulog \
	libvideo-metadata \
	sdl2

ifeq ("$(TARGET_OS)-$(TARGET_OS_FLAVOUR)","linux-native")
  LOCAL_LIBRARIES += \
	glfw3 \
	opengl
else ifeq ("$(TARGET_OS)-$(TARGET_OS_FLAVOUR)","darwin-native")
  LOCAL_LDLIBS += \
	-framework OpenGL
  LOCAL_LIBRARIES += \
	glfw3
endif

include $(BUILD_EXECUTABLE)
