
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
LOCAL_CFLAGS := -DPDRAW_API_EXPORTS -fvisibility=hidden -D_USE_MATH_DEFINES -D_GNU_SOURCE

LOCAL_SRC_FILES := \
	src/pdraw_channel_coded_video.cpp \
	src/pdraw_channel_raw_video.cpp \
	src/pdraw_decoder_video.cpp \
	src/pdraw_demuxer.cpp \
	src/pdraw_demuxer_record.cpp \
	src/pdraw_demuxer_stream_mux.cpp \
	src/pdraw_demuxer_stream_net.cpp \
	src/pdraw_demuxer_stream.cpp \
	src/pdraw_element.cpp \
	src/pdraw_encoder_video.cpp \
	src/pdraw_external_coded_video_sink.cpp \
	src/pdraw_external_raw_video_sink.cpp \
	src/pdraw_gles2_hmd_colors.cpp \
	src/pdraw_gles2_hmd_indices.cpp \
	src/pdraw_gles2_hmd_positions_cockpitglasses.cpp \
	src/pdraw_gles2_hmd_positions_cockpitglasses2.cpp \
	src/pdraw_gles2_hmd_shaders.cpp \
	src/pdraw_gles2_hmd_texcoords_cockpitglasses_blue.cpp \
	src/pdraw_gles2_hmd_texcoords_cockpitglasses_red.cpp \
	src/pdraw_gles2_hmd_texcoords.cpp \
	src/pdraw_gles2_hmd.cpp \
	src/pdraw_gles2_video.cpp \
	src/pdraw_media.cpp \
	src/pdraw_muxer_record.cpp \
	src/pdraw_muxer_stream_rtmp.cpp \
	src/pdraw_muxer.cpp \
	src/pdraw_renderer_gles2.cpp \
	src/pdraw_renderer_videocoreegl.cpp \
	src/pdraw_renderer.cpp \
	src/pdraw_scaler_video.cpp \
	src/pdraw_session.cpp \
	src/pdraw_settings.cpp \
	src/pdraw_sink_coded_video.cpp \
	src/pdraw_sink_raw_video.cpp \
	src/pdraw_source_coded_video.cpp \
	src/pdraw_source_raw_video.cpp \
	src/pdraw_utils.cpp \
	src/pdraw_wrapper.cpp

LOCAL_LIBRARIES := \
	eigen \
	libfutils \
	libh264 \
	libh265 \
	libmedia-buffers \
	libmedia-buffers-memory \
	libmedia-buffers-memory-generic \
	libmp4 \
	libpomp \
	librtp \
	librtsp \
	libsdp \
	libtransport-packet \
	libtransport-socket \
	libulog \
	libvideo-decode \
	libvideo-defs \
	libvideo-encode \
	libvideo-metadata \
	libvideo-scale \
	libvideo-streaming
LOCAL_CONDITIONAL_LIBRARIES := \
	OPTIONAL:json \
	OPTIONAL:libmux \
	OPTIONAL:librtmp

ifeq ($(TARGET_CPU),$(filter %$(TARGET_CPU),s905d3 s905x3))
  LOCAL_CFLAGS += -DUSE_GLES2
  LOCAL_LDLIBS += -lGLESv2
  LOCAL_LIBRARIES += \
	am-gpu
else ifeq ("$(TARGET_OS)-$(TARGET_OS_FLAVOUR)","linux-native")
  LOCAL_CFLAGS += -DUSE_GLES2 -DUSE_GLFW3
  LOCAL_LIBRARIES += \
	glfw3 \
	opengl
else ifeq ("$(TARGET_OS)-$(TARGET_OS_FLAVOUR)","linux-android")
  LOCAL_CFLAGS += -DUSE_GLES2
  LOCAL_LDLIBS += -lEGL -lGLESv2 -landroid
else ifeq ("$(TARGET_OS)-$(TARGET_OS_FLAVOUR)","darwin-native")
  LOCAL_CFLAGS += -DUSE_GLES2 -DUSE_GLFW3
  LOCAL_LDLIBS += \
	-framework OpenGL
  LOCAL_LIBRARIES += \
	glfw3
else ifeq ("$(TARGET_OS)-$(TARGET_OS_FLAVOUR)","darwin-iphoneos")
  LOCAL_CFLAGS += -DUSE_GLES2
  LOCAL_LDLIBS += \
	-framework OpenGLES
else ifeq ("$(TARGET_OS)","windows")
  LOCAL_CFLAGS += -DUSE_GLES2 -D_WIN32_WINNT=0x0600 -DEPOXY_SHARED
  LOCAL_LDLIBS += -lws2_32 -lepoxy
endif

include $(BUILD_LIBRARY)
