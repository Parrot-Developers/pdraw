
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
LOCAL_EXPORT_LDLIBS +=	-lavahi-client -lavahi-common
include $(BUILD_PREBUILT)
