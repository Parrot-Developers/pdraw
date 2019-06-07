
LOCAL_PATH := $(call my-dir)

ifeq ("$(TARGET_OS)","linux")
ifneq ("$(TARGET_OS_FLAVOUR)","android")

include $(CLEAR_VARS)

LOCAL_MODULE := qpdraw
LOCAL_CATEGORY_PATH := libs
LOCAL_DESCRIPTION := Parrot Drones Awesome Video Viewer Qt library
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/include
LOCAL_CXXFLAGS := -std=c++11
LOCAL_LIBRARIES := \
	libpdraw-backend \
	libulog
LOCAL_DEPENDS_MODULES := qt5-base
LOCAL_EXPORT_LDLIBS := -lqpdraw

include $(BUILD_QMAKE)

endif
endif
