LOCAL_PATH := $(call my-dir)

ifeq ($(NDK_DEBUG),1)
openni_conf := debug
else ifeq ($(NDK_DEBUG),true)
openni_conf := debug
else
openni_conf := release
endif

# OpenNI =============================================
include $(CLEAR_VARS)
LOCAL_MODULE := OpenNI

LOCAL_SRC_FILES := lib/$(openni_conf)/$(TARGET_ARCH_ABI)/libOpenNI.so
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/include
LOCAL_SHARED_LIBRARIES := libusb-1.0

include $(PREBUILT_SHARED_LIBRARY)
#=====================================================

# OpenNI.jni =============================================
include $(CLEAR_VARS)
LOCAL_MODULE := OpenNI.jni

LOCAL_SRC_FILES := lib/$(openni_conf)/$(TARGET_ARCH_ABI)/libOpenNI.jni.so
LOCAL_SHARED_LIBRARIES := OpenNI

include $(PREBUILT_SHARED_LIBRARY)
#=========================================================

# nimCodecs =============================================
include $(CLEAR_VARS)
LOCAL_MODULE := nimCodecs

LOCAL_SRC_FILES := lib/$(openni_conf)/$(TARGET_ARCH_ABI)/libnimCodecs.so
LOCAL_SHARED_LIBRARIES := OpenNI

include $(PREBUILT_SHARED_LIBRARY)
#========================================================

# nimMockNodes =============================================
include $(CLEAR_VARS)
LOCAL_MODULE := nimMockNodes

LOCAL_SRC_FILES := lib/$(openni_conf)/$(TARGET_ARCH_ABI)/libnimMockNodes.so
LOCAL_SHARED_LIBRARIES := OpenNI

include $(PREBUILT_SHARED_LIBRARY)
#===========================================================

# nimRecorder =============================================
include $(CLEAR_VARS)
LOCAL_MODULE := nimRecorder

LOCAL_SRC_FILES := lib/$(openni_conf)/$(TARGET_ARCH_ABI)/libnimRecorder.so
LOCAL_SHARED_LIBRARIES := OpenNI

include $(PREBUILT_SHARED_LIBRARY)
#==========================================================

$(call import-module,libusb-1.0)
