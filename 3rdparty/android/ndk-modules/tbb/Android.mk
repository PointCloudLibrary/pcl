LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE := tbb

LOCAL_SRC_FILES := lib/$(TARGET_ARCH_ABI)/libtbb.so
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/include

include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := tbbmalloc

LOCAL_SRC_FILES := lib/$(TARGET_ARCH_ABI)/libtbbmalloc.so
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/include

include $(PREBUILT_SHARED_LIBRARY)
