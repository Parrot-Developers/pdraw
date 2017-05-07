
TARGET_OS := linux
TARGET_OS_FLAVOUR := native
TARGET_GLOBAL_CFLAGS += -std=gnu99

ARSDK_BUILD_FOR_APP := 1

# Use our own json, ncurses version
prebuilt.json.override := 1
prebuilt.ncurses.override := 1
