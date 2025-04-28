
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE := libpdraw-gles2hud
LOCAL_CATEGORY_PATH := libs
LOCAL_DESCRIPTION := PDrAW OpenGL ES 2.0 HUD rendering library
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/include
# Public API headers - top level headers first
# This header list is currently used to generate a python binding
LOCAL_EXPORT_CUSTOM_VARIABLES := LIBPDRAW_GLES2HUD_HEADERS=$\
	$(LOCAL_PATH)/include/pdraw/pdraw_gles2hud.h;
LOCAL_CFLAGS := \
	-DPDRAW_GLES2HUD_API_EXPORTS \
	-fvisibility=hidden \
	-std=gnu99 \
	-D_USE_MATH_DEFINES
LOCAL_CXXFLAGS := -std=c++11
LOCAL_SRC_FILES := \
	src/pdraw_gles2hud.cpp \
	src/pdraw_gles2hud_icons.cpp \
	src/pdraw_gles2hud_icons_data.cpp \
	src/pdraw_gles2hud_instruments.cpp \
	src/pdraw_gles2hud_shaders.cpp \
	src/pdraw_gles2hud_shapes.cpp \
	src/pdraw_gles2hud_text.cpp \
	src/pdraw_gles2hud_text_profont36.cpp
LOCAL_LIBRARIES := \
	eigen \
	libpdraw \
	libulog \
	libvideo-metadata

ifeq ($(TARGET_CPU),$(filter %$(TARGET_CPU),s905d3 s905x3))
  LOCAL_LDLIBS += -lGLESv2
  LOCAL_LIBRARIES += \
	am-gpu
else ifeq ($(TARGET_CPU),qrb5165)
  LOCAL_LIBRARIES += \
	glesv2
else ifeq ($(TARGET_CPU),qcs405)
  LOCAL_LIBRARIES += \
	glesv2 \
	egl
else ifeq ("$(TARGET_OS)-$(TARGET_OS_FLAVOUR)","linux-native")
  LOCAL_LIBRARIES += \
	opengl
else ifeq ("$(TARGET_OS)-$(TARGET_OS_FLAVOUR)","linux-android")
  LOCAL_LDLIBS += -lGLESv2
else ifeq ("$(TARGET_OS)-$(TARGET_OS_FLAVOUR)","darwin-native")
  LOCAL_LDLIBS += \
	-framework OpenGL
else ifeq ($(TARGET_OS_FLAVOUR),$(filter %$(TARGET_OS_FLAVOUR),iphoneos iphonesimulator))
  LOCAL_LDLIBS += \
	-framework OpenGLES
else ifeq ("$(TARGET_OS)","windows")
  LOCAL_CFLAGS += -D_WIN32_WINNT=0x0600 -DEPOXY_SHARED
  LOCAL_LDLIBS += -lepoxy
endif

include $(BUILD_LIBRARY)
