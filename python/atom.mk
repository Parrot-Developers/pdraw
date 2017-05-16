LOCAL_PATH := $(call my-dir)

ifneq ("$(shell which python-config)","")
ifneq ("$(shell which swig)","")

include $(CLEAR_VARS)

LOCAL_MODULE := pdraw_python
LOCAL_DESCRIPTION := Pdraw wrapper to python using swig
LOCAL_CATEGORY_PATH := multimedia

LOCAL_LIBRARIES := libpdraw

PDRAW_PYTHON_BUILD_DIR := $(call local-get-build-dir)
PDRAW_PYTHON_SWIG_WRAPPER := _pdraw_python.cpp
PDRAW_PATH=$(LOCAL_PATH)/../libpdraw

LOCAL_C_INCLUDES := $(shell find $(PDRAW_PATH)/include $(PDRAW_PATH)/src -type d)

PDRAW_PYTHON_SWIG_C_INCLUDES := $(foreach f, $(LOCAL_C_INCLUDES), -I$(f))

LOCAL_GENERATED_SRC_FILES := $(PDRAW_PYTHON_SWIG_WRAPPER)
LOCAL_MODULE_FILENAME := _$(LOCAL_MODULE).so
LOCAL_DESTDIR := usr/lib/python

LOCAL_CXXFLAGS := -std=c++11
LOCAL_CXXFLAGS += $(shell python-config --includes)

LOCAL_LDLIBS := $(shell python-config --ldflags)

PDRAW_PYTHON_SWIG_SOURCE := $(LOCAL_MODULE).i
PDRAW_PYTHON_NAME := $(LOCAL_MODULE)

PDRAW_PYTHON_SWIG_DEPENDS := $(foreach d, $(LOCAL_C_INCLUDES), $(d)/*) $(shell find $(LOCAL_PATH)/*i)

$(PDRAW_PYTHON_BUILD_DIR)/$(PDRAW_PYTHON_SWIG_WRAPPER): $(LOCAL_PATH)/$(PDRAW_PYTHON_SWIG_SOURCE) $(PDRAW_PYTHON_SWIG_DEPENDS)
	$(call print-banner1,"$(PRIVATE_MODE_MSG)Swig",$(PDRAW_PYTHON_SWIG_WRAPPER),$(PDRAW_PYTHON_SWIG_SOURCE))
	$(Q) swig -python -c++ $(PDRAW_PYTHON_SWIG_C_INCLUDES) -o $@ $<
	$(Q) cp $(PDRAW_PYTHON_BUILD_DIR)/$(PDRAW_PYTHON_NAME).py $(TARGET_OUT_STAGING)/usr/lib/python/


include $(BUILD_SHARED_LIBRARY)

endif
endif
