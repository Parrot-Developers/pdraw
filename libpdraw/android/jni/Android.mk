LOCAL_PATH := $(call my-dir)

# Include makefiles here. Its important that these 
# includes are done after the main module, explanation below.

# create a temp variable with the current path, because it 
# changes after each include
ZPATH := $(LOCAL_PATH)

include $(PRODUCT_OUT_DIR)/$(TARGET_ARCH_ABI)/sdk/Android.mk

include $(ZPATH)/../../../../libARSAL/Android.mk
include $(ZPATH)/../../../../libARNetworkAL/Android.mk
include $(ZPATH)/../../../../libmux/android/src/main/jni/Android.mk
include $(ZPATH)/../../../../libARNetwork/Android.mk
include $(ZPATH)/../../../../libARCommands/Android.mk
include $(ZPATH)/../../../../libARDiscovery/Android.mk
include $(ZPATH)/../src/main/jni/Android.mk