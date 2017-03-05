
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := pdraw_linux
LOCAL_DESCRIPTION := Parrot Drones Awesome Video Viewer Linux Application
LOCAL_CATEGORY_PATH := multimedia
LOCAL_SRC_FILES := pdraw_linux.c
LOCAL_LIBRARIES := libpdraw libulog libARCommands libARNetwork libARNetworkAL libARDiscovery json curl
LOCAL_CFLAGS += $(shell sdl-config --cflags) -DUSE_SDL
LOCAL_LDLIBS += $(shell sdl-config --libs)

include $(BUILD_EXECUTABLE)
