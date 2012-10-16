LOCAL_PATH := $(call my-dir)

# Header-only Boost libraries ===================
include $(CLEAR_VARS)
LOCAL_MODULE := boost

LOCAL_SRC_FILES := lib/$(TARGET_ARCH_ABI)/dummy.a
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/include

include $(PREBUILT_STATIC_LIBRARY)
#================================================

# Boost.Filesystem ============================================
include $(CLEAR_VARS)
LOCAL_MODULE := boost_filesystem

LOCAL_SRC_FILES := lib/$(TARGET_ARCH_ABI)/libboost_filesystem.a
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/include
LOCAL_STATIC_LIBRARIES := boost_system

include $(PREBUILT_STATIC_LIBRARY)
#==============================================================

# Boost.System ============================================
include $(CLEAR_VARS)
LOCAL_MODULE := boost_system

LOCAL_SRC_FILES := lib/$(TARGET_ARCH_ABI)/libboost_system.a
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/include

include $(PREBUILT_STATIC_LIBRARY)
#==========================================================
