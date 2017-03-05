
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := libpdraw
LOCAL_DESCRIPTION := Parrot Drones Awesome Video Viewer library
LOCAL_CATEGORY_PATH := libs
LOCAL_SRC_FILES := \
	src/pdraw.cpp \
	src/pdraw_wrapper.cpp \
	src/pdraw_bufferqueue.cpp \
	src/pdraw_demuxer_stream.cpp \
	src/pdraw_demuxer_record.cpp \
	src/pdraw_utils.cpp \
	src/pdraw_metadata.cpp \
	src/pdraw_avcdecoder.cpp \
	src/pdraw_avcdecoder_ffmpeg.cpp \
	src/pdraw_gles2_hud.cpp \
	src/pdraw_gles2_video.cpp \
	src/pdraw_renderer.cpp \
	src/pdraw_renderer_null.cpp \
	src/pdraw_renderer_gles2.cpp
LOCAL_EXPORT_CXXFLAGS := -Wextra -std=c++0x
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/include
LOCAL_LIBRARIES := libulog libpomp libvideo-metadata libARStream2 libmp4

LOCAL_CFLAGS += -DUSE_FFMPEG -DUSE_GLES2
LOCAL_LDLIBS += -lavcodec -lavdevice -lavfilter -lavformat -lswresample -lswscale -lavutil -lGL -lGLU

include $(BUILD_LIBRARY)
