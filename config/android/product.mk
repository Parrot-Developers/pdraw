
TARGET_OS := linux
TARGET_OS_FLAVOUR := android
TARGET_LIBC := bionic

TARGET_ANDROID_APILEVEL := 21

# Setup arch/cpu based on abi
ifeq ("$(ANDROID_ABI)","armeabi")
  TARGET_ARCH := arm
else ifeq ("$(ANDROID_ABI)","armeabi-v7a")
  TARGET_ARCH := arm
  TARGET_CPU := armv7a
else ifeq ("$(ANDROID_ABI)","mips")
  TARGET_ARCH := mips
  TARGET_CPU :=
else ifeq ("$(ANDROID_ABI)","x86")
  TARGET_ARCH := x86
  TARGET_CPU :=
else ifeq ("$(ANDROID_ABI)","arm64-v8a")
  TARGET_ARCH := aarch64
  TARGET_CPU :=
else ifeq ("$(ANDROID_ABI)","")
  $(warning Missing ANDROID_ABI using armeabi-v7a)
  TARGET_ARCH := arm
  TARGET_CPU := armv7a
else
  $(error Invalid ANDROID_ABI: $(ANDROID_ABI))
endif

ifndef ANDROID_NDK_PATH
  $(error Missing ANDROID_NDK_PATH)
endif

ifndef ANDROID_SDK_PATH
  $(error Missing ANDROID_SDK_PATH)
endif

TARGET_ANDROID_NDK := $(ANDROID_NDK_PATH)
TARGET_ANDROID_SDK := $(ANDROID_SDK_PATH)

TARGET_DEFAULT_LIB_DESTDIR := usr/lib

TARGET_GLOBAL_CFLAGS += -std=gnu99

ARSDK_BUILD_FOR_APP := 1
