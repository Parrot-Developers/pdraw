
LOCAL_PATH := $(call my-dir)

ifeq ("$(TARGET_OS)-$(TARGET_OS_FLAVOUR)","darwin-native")

include $(CLEAR_VARS)

LOCAL_MODULE := pdraw
LOCAL_DESCRIPTION := Parrot Drones Awesome Video Viewer MacOS Application
LOCAL_CATEGORY_PATH := multimedia
LOCAL_SRC_FILES := pdraw_macos.c
LOCAL_LIBRARIES := \
	libpdraw \
	libulog \
	libARCommands \
	libARNetwork \
	libARNetworkAL \
	libARDiscovery \
	json \
	curl
LOCAL_CONDITIONAL_LIBRARIES := \
	OPTIONAL:sdl2

include $(BUILD_EXECUTABLE)

endif
