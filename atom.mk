
PDRAW_LOCAL_PATH := $(call my-dir)

include $(PDRAW_LOCAL_PATH)/libpdraw/atom.mk

ifeq ("$(TARGET_OS)","linux")
	ifeq ("$(TARGET_OS_FLAVOUR)","native")
		include $(PDRAW_LOCAL_PATH)/apps/pdraw_linux/atom.mk
	endif
endif

ifeq ("$(TARGET_OS)","linux")
	ifeq ("$(TARGET_OS_FLAVOUR)","generic")
		ifeq ("$(TARGET_PRODUCT_VARIANT)","raspi")
			include $(PDRAW_LOCAL_PATH)/apps/pdraw_raspi/atom.mk
		endif
	endif
endif
