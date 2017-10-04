
# Override alchemy default AR
TARGET_AR := $(shell xcrun --find --sdk iphoneos ar)

TARGET_OS := darwin
TARGET_OS_FLAVOUR := iphoneos
TARGET_ARCH := arm

TARGET_FORCE_STATIC := 1
TARGET_IPHONE_VERSION := 7.0

TARGET_GLOBAL_CFLAGS += -fembed-bitcode
TARGET_GLOBAL_OBJCFLAGS += -fobjc-arc

ARSDK_BUILD_FOR_APP := 1
