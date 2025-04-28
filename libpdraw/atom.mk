
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := libpdraw
LOCAL_DESCRIPTION := Parrot Drones Audio and Video Vector library
LOCAL_CATEGORY_PATH := libs

LOCAL_CONFIG_FILES := pdraw.in
$(call load-config)

LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/include
# Public API headers - top level headers first
# This header list is currently used to generate a python binding
LOCAL_EXPORT_CUSTOM_VARIABLES := LIBPDRAW_HEADERS=$\
	$(LOCAL_PATH)/include/pdraw/pdraw.h:$\
	$(LOCAL_PATH)/include/pdraw/pdraw_defs.h;
LOCAL_EXPORT_CXXFLAGS := -std=c++11
LOCAL_CFLAGS := -DPDRAW_API_EXPORTS -fvisibility=hidden -D_USE_MATH_DEFINES -D_GNU_SOURCE

LOCAL_SRC_FILES := \
	src/pdraw_alsa_audio.cpp \
	src/pdraw_alsa_source.cpp \
	src/pdraw_channel.cpp \
	src/pdraw_channel_audio.cpp \
	src/pdraw_channel_coded_video.cpp \
	src/pdraw_channel_raw_video.cpp \
	src/pdraw_decoder_audio.cpp \
	src/pdraw_decoder_video.cpp \
	src/pdraw_demuxer_record_audio_media.cpp \
	src/pdraw_demuxer_record_coded_video_media.cpp \
	src/pdraw_demuxer_record_raw_video_media.cpp \
	src/pdraw_demuxer_record.cpp \
	src/pdraw_demuxer_stream_mux.cpp \
	src/pdraw_demuxer_stream_net.cpp \
	src/pdraw_demuxer_stream.cpp \
	src/pdraw_demuxer.cpp \
	src/pdraw_element.cpp \
	src/pdraw_encoder_audio.cpp \
	src/pdraw_encoder_video.cpp \
	src/pdraw_external_audio_sink.cpp \
	src/pdraw_external_audio_source.cpp \
	src/pdraw_external_coded_video_sink.cpp \
	src/pdraw_external_coded_video_source.cpp \
	src/pdraw_external_raw_video_sink.cpp \
	src/pdraw_external_raw_video_source.cpp \
	src/pdraw_gl_video.cpp \
	src/pdraw_media.cpp \
	src/pdraw_muxer_record.cpp \
	src/pdraw_muxer_stream_rtmp.cpp \
	src/pdraw_muxer.cpp \
	src/pdraw_renderer_audio_alsa.cpp \
	src/pdraw_renderer_audio.cpp \
	src/pdraw_renderer_video.cpp \
	src/pdraw_renderer_video_gl.cpp \
	src/pdraw_scaler_video.cpp \
	src/pdraw_session.cpp \
	src/pdraw_settings.cpp \
	src/pdraw_sink.cpp \
	src/pdraw_source.cpp \
	src/pdraw_utils.cpp \
	src/pdraw_video_pres_stats.cpp \
	src/pdraw_vipc_source.cpp \
	src/pdraw_wrapper.cpp

LOCAL_LIBRARIES := \
	eigen \
	libaac \
	libaudio-decode \
	libaudio-defs \
	libaudio-encode \
	libaudio-encode-core \
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
	libvideo-encode-core \
	libvideo-metadata \
	libvideo-scale \
	libvideo-scale-core \
	libvideo-streaming

LOCAL_CONDITIONAL_LIBRARIES := \
	OPTIONAL:json \
	OPTIONAL:libmux \
	OPTIONAL:librtmp \
	OPTIONAL:libvideo-ipc \
	OPTIONAL:libvideo-ipc-client-config

LOCAL_CONDITIONAL_LIBRARIES += \
	CONFIG_PDRAW_VIPC_BACKEND_DMABUF:libvideo-ipc-dmabuf-be \
	CONFIG_PDRAW_VIPC_BACKEND_DMABUF:libmedia-buffers-memory-ion \
	CONFIG_PDRAW_VIPC_BACKEND_HISI:libvideo-ipc-hisibe \
	CONFIG_PDRAW_VIPC_BACKEND_HISI:libmedia-buffers-memory-hisi \
	CONFIG_PDRAW_VIPC_BACKEND_NETWORK_CBUF:libvideo-ipc-network-cbuf-be \
	CONFIG_PDRAW_VIPC_BACKEND_NETWORK_HISI:libvideo-ipc-network-hisi-be \
	CONFIG_PDRAW_VIPC_BACKEND_SHM:libvideo-ipc-shmbe \
	CONFIG_PDRAW_USE_ALSA:alsa-lib

ifeq ("$(TARGET_OS)","windows")
  LOCAL_CFLAGS += -D_WIN32_WINNT=0x0600
  LOCAL_LDLIBS += -lws2_32
endif

ifdef CONFIG_PDRAW_USE_GL
  ifeq ($(TARGET_CPU),$(filter %$(TARGET_CPU),s905d3 s905x3))
    LOCAL_LDLIBS += -lGLESv2
    LOCAL_CONDITIONAL_LIBRARIES += \
	CONFIG_PDRAW_USE_GL:am-gpu
  else ifeq ($(TARGET_CPU),qcs405)
    LOCAL_CFLAGS += -DUSE_GLES2
    LOCAL_LIBRARIES += \
	glesv2 \
	egl
  else ifeq ("$(TARGET_OS)-$(TARGET_OS_FLAVOUR)","linux-native")
    LOCAL_CONDITIONAL_LIBRARIES += \
	CONFIG_PDRAW_USE_GL:opengl
  else ifeq ("$(TARGET_OS)-$(TARGET_OS_FLAVOUR)","linux-android")
    LOCAL_LDLIBS += -lEGL -lGLESv2 -landroid
  else ifeq ("$(TARGET_OS)-$(TARGET_OS_FLAVOUR)","darwin-native")
    LOCAL_LDLIBS += -framework OpenGL
  else ifeq ($(TARGET_OS_FLAVOUR),$(filter %$(TARGET_OS_FLAVOUR),iphoneos iphonesimulator))
    LOCAL_LDLIBS += -framework OpenGLES
  else ifeq ("$(TARGET_OS)","windows")
    LOCAL_CFLAGS += -DEPOXY_SHARED
    LOCAL_LDLIBS += -lepoxy
  endif
endif

include $(BUILD_LIBRARY)
