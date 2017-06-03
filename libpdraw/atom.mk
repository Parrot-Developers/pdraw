
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := libpdraw
LOCAL_DESCRIPTION := Parrot Drones Awesome Video Viewer library
LOCAL_CATEGORY_PATH := libs
LOCAL_SRC_FILES := \
	src/pdraw_impl.cpp \
	src/pdraw_wrapper.cpp \
	src/pdraw_buffer.cpp \
	src/pdraw_session.cpp \
	src/pdraw_media_video.cpp \
	src/pdraw_demuxer_stream.cpp \
	src/pdraw_demuxer_record.cpp \
	src/pdraw_utils.cpp \
	src/pdraw_metadata_session.cpp \
	src/pdraw_metadata_videoframe.cpp \
	src/pdraw_avcdecoder.cpp \
	src/pdraw_avcdecoder_ffmpeg.cpp \
	src/pdraw_avcdecoder_videocoreomx.cpp \
	src/pdraw_gles2_hud.cpp \
	src/pdraw_gles2_video.cpp \
	src/pdraw_renderer.cpp \
	src/pdraw_renderer_null.cpp \
	src/pdraw_renderer_videocoreegl.cpp \
	src/pdraw_renderer_gles2.cpp \
	src/pdraw_filter_videoframe.cpp
LOCAL_EXPORT_CXXFLAGS := -Wextra -std=c++0x
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/include
LOCAL_LIBRARIES := libulog libpomp libvideo-metadata libARStream2 libmp4 libh264 json

ifeq ("$(TARGET_OS)","linux")
	ifeq ("$(TARGET_OS_FLAVOUR)","native")
		LOCAL_CFLAGS += -DUSE_FFMPEG -DUSE_GLES2
		LOCAL_LDLIBS += -lavcodec -lavdevice -lavfilter -lavformat -lswresample -lswscale -lavutil -lGL -lGLU
	else ifeq ("$(TARGET_OS_FLAVOUR)","generic")
		ifeq ("$(TARGET_PRODUCT_VARIANT)","raspi")
			LOCAL_LIBRARIES += ilclient
			LOCAL_CFLAGS += -DBCM_VIDEOCORE -DUSE_VIDEOCOREOMX -DUSE_VIDEOCOREEGL -DUSE_GLES2 -I$(SDKSTAGE)/opt/vc/include
			LOCAL_CFLAGS += -DSTANDALONE -D__STDC_CONSTANT_MACROS -D__STDC_LIMIT_MACROS -DTARGET_POSIX -D_LINUX -fPIC -DPIC -D_REENTRANT -D_LARGEFILE64_SOURCE -D_FILE_OFFSET_BITS=64 -U_FORTIFY_SOURCE -Wall -g -DHAVE_LIBOPENMAX=2 -DOMX -DOMX_SKIP64BIT -ftree-vectorize -pipe -DUSE_EXTERNAL_OMX -DHAVE_LIBBCM_HOST -DUSE_EXTERNAL_LIBBCM_HOST -DUSE_VCHIQ_ARM -Wno-psabi
			LOCAL_LDLIBS += -L$(SDKSTAGE)/opt/vc/lib -lbrcmGLESv2 -lbrcmEGL -lopenmaxil -lbcm_host -lvcos -lvchiq_arm -lpthread -lrt -lm
		endif
	endif
endif

include $(BUILD_LIBRARY)
