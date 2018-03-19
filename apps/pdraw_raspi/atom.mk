
LOCAL_PATH := $(call my-dir)

ifeq ("$(TARGET_OS)-$(TARGET_OS_FLAVOUR)-$(TARGET_PRODUCT_VARIANT)","linux-generic-raspi")

include $(CLEAR_VARS)

LOCAL_MODULE := pdraw
LOCAL_DESCRIPTION := Parrot Drones Awesome Video Viewer RaspberryPi Application
LOCAL_CATEGORY_PATH := multimedia
LOCAL_SRC_FILES := pdraw_raspi.c
LOCAL_LIBRARIES := \
	libpdraw \
	mmal \
	libulog \
	libARCommands \
	libARNetwork \
	libARNetworkAL \
	libARDiscovery \
	json \
	curl \
	libpng
LOCAL_LDLIBS += -lbrcmGLESv2 -lbrcmEGL -lbcm_host -lvchiq_arm \
	-lavahi-client -lavahi-common -ldbus-1

include $(BUILD_EXECUTABLE)

endif
