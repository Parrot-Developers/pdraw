
PDRAW_LOCAL_PATH := $(call my-dir)

include $(PDRAW_LOCAL_PATH)/libpdraw/atom.mk

ifeq ("$(TARGET_OS)-$(TARGET_OS_FLAVOUR)","linux-native")
	include $(PDRAW_LOCAL_PATH)/config/pclinux/deps.mk
	include $(PDRAW_LOCAL_PATH)/apps/pdraw_linux/atom.mk
endif

ifeq ("$(TARGET_OS)-$(TARGET_OS_FLAVOUR)-$(TARGET_PRODUCT_VARIANT)", \
	"linux-generic-raspi")
	include $(PDRAW_LOCAL_PATH)/config/raspi/deps.mk
	include $(PDRAW_LOCAL_PATH)/apps/pdraw_raspi/atom.mk
endif

include $(PDRAW_LOCAL_PATH)/python/atom.mk
