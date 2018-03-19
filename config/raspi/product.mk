
CONFIG_DIR := $(call my-dir)

TARGET_OS := linux
TARGET_OS_FLAVOUR := generic
TARGET_ARCH :=  arm
TARGET_DEFAULT_ARM_MODE := arm
TARGET_CROSS := /opt/arm-rpi-4.9.3-linux-gnueabihf/bin/arm-linux-gnueabihf-
TARGET_LIBC := eglibc
TARGET_FLOAT_ABI := hard
export SDKSTAGE := $(CONFIG_DIR)/../../../../../raspberrypi-sdk
TARGET_GLOBAL_CFLAGS += -DRASPI -I$(SDKSTAGE)/usr/include -I$(SDKSTAGE)/opt/vc/include
TARGET_GLOBAL_LDLIBS += -L$(SDKSTAGE)/usr/lib -L$(SDKSTAGE)/opt/vc/lib

ARSDK_BUILD_FOR_APP := 1

# Use our own json, ncurses version
prebuilt.json.override := 1
prebuilt.ncurses.override := 1
