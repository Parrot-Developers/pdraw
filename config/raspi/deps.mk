
LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)
LOCAL_MODULE := zlib
LOCAL_DESCRIPTION := Zlib compression library
LOCAL_EXPORT_LDLIBS += -lz
include $(BUILD_PREBUILT)


LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)
LOCAL_MODULE := libpng
LOCAL_DESCRIPTION := PNG reference library
LOCAL_LIBRARIES := zlib
LOCAL_EXPORT_LDLIBS += -lpng
include $(BUILD_PREBUILT)


LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)
LOCAL_MODULE := avahi
LOCAL_DESCRIPTION := Avahi service discovery suite
LOCAL_EXPORT_LDLIBS += -lavahi-client -lavahi-common
include $(BUILD_PREBUILT)


LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)
LOCAL_MODULE := mmal
LOCAL_DESCRIPTION := Broadcom VideoCore Multi-Media Abstraction Layer
LOCAL_EXPORT_LDLIBS += -Wl,--no-as-needed -lmmal -lmmal_core -lmmal_components \
	-lmmal_util -lmmal_vc_client -lvcos -lvcsm -lcontainers -Wl,--as-needed
include $(BUILD_PREBUILT)
