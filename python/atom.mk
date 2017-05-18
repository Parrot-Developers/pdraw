LOCAL_PATH := $(call my-dir)

ifneq ("$(shell which python-config)","")
ifneq ("$(shell which swig)","")

include $(CLEAR_VARS)

LOCAL_MODULE := pdraw_python
LOCAL_DESCRIPTION := Pdraw wrapper to python using swig
LOCAL_CATEGORY_PATH := multimedia

LOCAL_LIBRARIES := libpdraw
LOCAL_CONFIG_FILES := Config.in
$(call load-config)

PDRAW_PYTHON_BUILD_DIR := $(call local-get-build-dir)
PDRAW_PYTHON_SWIG_WRAPPER := _pdraw_python.cpp
PDRAW_PATH=$(LOCAL_PATH)/../libpdraw

LOCAL_C_INCLUDES := $(shell find $(PDRAW_PATH)/include $(PDRAW_PATH)/src -type d)

PDRAW_PYTHON_SWIG_C_INCLUDES := $(foreach f, $(LOCAL_C_INCLUDES), -I$(f))

LOCAL_GENERATED_SRC_FILES := $(PDRAW_PYTHON_SWIG_WRAPPER)
LOCAL_MODULE_FILENAME := _$(LOCAL_MODULE).so
LOCAL_DESTDIR := usr/lib/python

PDRAW_PYTHON_DESTDIR := $(TARGET_OUT_STAGING)/usr/lib/python

ifeq ($(CONFIG_PDRAW_PYTHON_PYTHON3),y)
PDRAW_PYTHON_PYTHONCONFIG := python3-config
else
PDRAW_PYTHON_PYTHONCONFIG := python-config
endif

LOCAL_CXXFLAGS := -std=c++11
LOCAL_CXXFLAGS += $(shell $(PDRAW_PYTHON_PYTHONCONFIG) --includes)

LOCAL_LDLIBS := $(shell $(PDRAW_PYTHON_PYTHONCONFIG) --ldflags)

PDRAW_PYTHON_SWIG_SOURCE := $(LOCAL_MODULE).i
PDRAW_PYTHON_NAME := $(LOCAL_MODULE)

PDRAW_PYTHON_SWIG_DEPENDS := $(foreach d, $(LOCAL_C_INCLUDES), $(d)/*) $(shell find $(LOCAL_PATH)/*i)

$(PDRAW_PYTHON_BUILD_DIR)/$(PDRAW_PYTHON_SWIG_WRAPPER): $(LOCAL_PATH)/$(PDRAW_PYTHON_SWIG_SOURCE) $(PDRAW_PYTHON_SWIG_DEPENDS)
	$(call print-banner1,"$(PRIVATE_MODE_MSG)Swig",$(PDRAW_PYTHON_SWIG_WRAPPER),$(PDRAW_PYTHON_SWIG_SOURCE))
	$(Q) swig -python -c++ $(PDRAW_PYTHON_SWIG_C_INCLUDES) -o $@ $<
	$(Q) mkdir -p $(PDRAW_PYTHON_DESTDIR)
	$(Q) cp $(PDRAW_PYTHON_BUILD_DIR)/$(PDRAW_PYTHON_NAME).py $(PDRAW_PYTHON_DESTDIR)


include $(BUILD_SHARED_LIBRARY)

endif
endif
