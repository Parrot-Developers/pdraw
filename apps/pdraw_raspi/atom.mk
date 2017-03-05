
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := pdraw_raspi
LOCAL_DESCRIPTION := Parrot Drones Awesome Video Viewer RaspberryPi Application
LOCAL_CATEGORY_PATH := multimedia
LOCAL_SRC_FILES := pdraw_raspi.c
LOCAL_LIBRARIES := libpdraw libulog libARCommands libARNetwork libARNetworkAL libARDiscovery json curl
LOCAL_CFLAGS += -I$(SDKSTAGE)/opt/vc/include -DSTANDALONE -D__STDC_CONSTANT_MACROS -D__STDC_LIMIT_MACROS -DTARGET_POSIX -D_LINUX -fPIC -DPIC -D_REENTRANT -D_LARGEFILE64_SOURCE -D_FILE_OFFSET_BITS=64 -U_FORTIFY_SOURCE -Wall -g -DHAVE_LIBOPENMAX=2 -DOMX -DOMX_SKIP64BIT -ftree-vectorize -pipe -DUSE_EXTERNAL_OMX -DHAVE_LIBBCM_HOST -DUSE_EXTERNAL_LIBBCM_HOST -DUSE_VCHIQ_ARM -Wno-psabi
LOCAL_LDLIBS += -L$(SDKSTAGE)/opt/vc/lib -lbrcmGLESv2 -lbrcmEGL -lopenmaxil -lbcm_host -lvcos -lvchiq_arm -lpthread -lrt -lm

include $(BUILD_EXECUTABLE)
