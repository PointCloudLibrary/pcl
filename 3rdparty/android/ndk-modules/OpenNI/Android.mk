LOCAL_PATH := $(call my-dir)

# OpenNI =============================================
include $(CLEAR_VARS)
LOCAL_MODULE := OpenNI

LOCAL_SRC_FILES := lib/$(TARGET_ARCH_ABI)/libOpenNI.so
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/include
LOCAL_SHARED_LIBRARIES := libusb-1.0

include $(PREBUILT_SHARED_LIBRARY)
#=====================================================

# OpenNI.jni =============================================
include $(CLEAR_VARS)
LOCAL_MODULE := OpenNI.jni

LOCAL_SRC_FILES := lib/$(TARGET_ARCH_ABI)/libOpenNI.jni.so
LOCAL_SHARED_LIBRARIES := OpenNI

include $(PREBUILT_SHARED_LIBRARY)
#=========================================================

# nimCodecs =============================================
include $(CLEAR_VARS)
LOCAL_MODULE := nimCodecs

LOCAL_SRC_FILES := lib/$(TARGET_ARCH_ABI)/libnimCodecs.so
LOCAL_SHARED_LIBRARIES := OpenNI

include $(PREBUILT_SHARED_LIBRARY)
#========================================================

# nimMockNodes =============================================
include $(CLEAR_VARS)
LOCAL_MODULE := nimMockNodes

LOCAL_SRC_FILES := lib/$(TARGET_ARCH_ABI)/libnimMockNodes.so
LOCAL_SHARED_LIBRARIES := OpenNI

include $(PREBUILT_SHARED_LIBRARY)
#===========================================================

# nimRecorder =============================================
include $(CLEAR_VARS)
LOCAL_MODULE := nimRecorder

LOCAL_SRC_FILES := lib/$(TARGET_ARCH_ABI)/libnimRecorder.so
LOCAL_SHARED_LIBRARIES := OpenNI

include $(PREBUILT_SHARED_LIBRARY)
#==========================================================

# XnCore =============================================
include $(CLEAR_VARS)
LOCAL_MODULE := XnCore

LOCAL_SRC_FILES := lib/$(TARGET_ARCH_ABI)/libXnCore.so
LOCAL_SHARED_LIBRARIES := OpenNI

include $(PREBUILT_SHARED_LIBRARY)
#=====================================================

# XnDDK =============================================
include $(CLEAR_VARS)
LOCAL_MODULE := XnDDK

LOCAL_SRC_FILES := lib/$(TARGET_ARCH_ABI)/libXnDDK.so
LOCAL_SHARED_LIBRARIES := OpenNI XnCore XnFormats

include $(PREBUILT_SHARED_LIBRARY)
#====================================================

# XnDeviceFile =============================================
include $(CLEAR_VARS)
LOCAL_MODULE := XnDeviceFile

LOCAL_SRC_FILES := lib/$(TARGET_ARCH_ABI)/libXnDeviceFile.so
LOCAL_SHARED_LIBRARIES := OpenNI XnCore XnFormats XnDDK

include $(PREBUILT_SHARED_LIBRARY)
#===========================================================

# XnDeviceSensorV2 =============================================
include $(CLEAR_VARS)
LOCAL_MODULE := XnDeviceSensorV2

LOCAL_SRC_FILES := lib/$(TARGET_ARCH_ABI)/libXnDeviceSensorV2.so
LOCAL_SHARED_LIBRARIES := OpenNI XnCore XnFormats XnDDK

include $(PREBUILT_SHARED_LIBRARY)
#===============================================================

# XnFormats =============================================
include $(CLEAR_VARS)
LOCAL_MODULE := XnFormats

LOCAL_SRC_FILES := lib/$(TARGET_ARCH_ABI)/libXnFormats.so
LOCAL_SHARED_LIBRARIES := OpenNI XnCore

include $(PREBUILT_SHARED_LIBRARY)
#===============================================================

$(call import-module,libusb-1.0)
