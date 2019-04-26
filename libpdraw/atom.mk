
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := libpdraw
LOCAL_DESCRIPTION := Parrot Drones Awesome Video Viewer library
LOCAL_CATEGORY_PATH := libs

LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/include
# Public API headers - top level headers first
# This header list is currently used to generate a python binding
LOCAL_EXPORT_CUSTOM_VARIABLES := LIBPDRAW_HEADERS=$\
	$(LOCAL_PATH)/include/pdraw/pdraw.h:$\
	$(LOCAL_PATH)/include/pdraw/pdraw_defs.h;
LOCAL_EXPORT_CXXFLAGS := -std=c++11
LOCAL_CFLAGS := -DPDRAW_API_EXPORTS -fvisibility=hidden

LOCAL_SRC_FILES := \
	src/pdraw_avcdecoder.cpp \
	src/pdraw_channel.cpp \
	src/pdraw_demuxer_record.cpp \
	src/pdraw_demuxer_stream.cpp \
	src/pdraw_demuxer_stream_mux.cpp \
	src/pdraw_demuxer_stream_net.cpp \
	src/pdraw_element.cpp \
	src/pdraw_gles2_hmd.cpp \
	src/pdraw_gles2_hmd_colors.cpp \
	src/pdraw_gles2_hmd_indices.cpp \
	src/pdraw_gles2_hmd_positions_cockpitglasses.cpp \
	src/pdraw_gles2_hmd_positions_cockpitglasses2.cpp \
	src/pdraw_gles2_hmd_shaders.cpp \
	src/pdraw_gles2_hmd_texcoords.cpp \
	src/pdraw_gles2_hmd_texcoords_cockpitglasses_blue.cpp \
	src/pdraw_gles2_hmd_texcoords_cockpitglasses_red.cpp \
	src/pdraw_gles2_video.cpp \
	src/pdraw_media.cpp \
	src/pdraw_metadata_session.cpp \
	src/pdraw_renderer.cpp \
	src/pdraw_renderer_gles2.cpp \
	src/pdraw_renderer_videocoreegl.cpp \
	src/pdraw_session.cpp \
	src/pdraw_settings.cpp \
	src/pdraw_sink.cpp \
	src/pdraw_sink_video.cpp \
	src/pdraw_socket_inet.cpp \
	src/pdraw_source.cpp \
	src/pdraw_utils.cpp \
	src/pdraw_wrapper.cpp

LOCAL_LIBRARIES := \
	eigen \
	libfutils \
	libh264 \
	libmp4 \
	libpomp \
	librtsp \
	libsdp \
	libulog \
	libvideo-buffers \
	libvideo-buffers-generic \
	libvideo-decode \
	libvideo-metadata \
	libvideo-streaming
LOCAL_CONDITIONAL_LIBRARIES := \
	OPTIONAL:json \
	OPTIONAL:libmux

ifeq ("$(TARGET_OS)-$(TARGET_OS_FLAVOUR)","linux-native")
  LOCAL_CFLAGS += -DUSE_GLES2
  LOCAL_LIBRARIES += \
	glfw3 \
	opengl
else ifeq ("$(TARGET_OS)-$(TARGET_OS_FLAVOUR)","linux-android")
  LOCAL_CFLAGS += -DUSE_GLES2
  LOCAL_LDLIBS += -lEGL -lGLESv2 -landroid
else ifeq ("$(TARGET_OS)-$(TARGET_OS_FLAVOUR)","darwin-native")
  LOCAL_CFLAGS += -DUSE_GLES2
  LOCAL_LDLIBS += \
	-framework OpenGL
  LOCAL_LIBRARIES += \
	glfw3
else ifeq ("$(TARGET_OS)-$(TARGET_OS_FLAVOUR)","darwin-iphoneos")
  LOCAL_CFLAGS += -DUSE_GLES2
  LOCAL_LDLIBS += \
	-framework OpenGLES
endif

include $(BUILD_LIBRARY)
