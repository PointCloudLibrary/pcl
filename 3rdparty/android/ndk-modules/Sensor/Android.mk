LOCAL_PATH := $(call my-dir)

ifeq ($(NDK_DEBUG),1)
sensor_conf := debug
else ifeq ($(NDK_DEBUG),true)
sensor_conf := debug
else
sensor_conf := release
endif

# XnCore =============================================
include $(CLEAR_VARS)
LOCAL_MODULE := XnCore

LOCAL_SRC_FILES := lib/$(sensor_conf)/$(TARGET_ARCH_ABI)/libXnCore.so
LOCAL_SHARED_LIBRARIES := OpenNI

include $(PREBUILT_SHARED_LIBRARY)
#=====================================================

# XnDDK =============================================
include $(CLEAR_VARS)
LOCAL_MODULE := XnDDK

LOCAL_SRC_FILES := lib/$(sensor_conf)/$(TARGET_ARCH_ABI)/libXnDDK.so
LOCAL_SHARED_LIBRARIES := OpenNI XnCore XnFormats

include $(PREBUILT_SHARED_LIBRARY)
#====================================================

# XnDeviceFile =============================================
include $(CLEAR_VARS)
LOCAL_MODULE := XnDeviceFile

LOCAL_SRC_FILES := lib/$(sensor_conf)/$(TARGET_ARCH_ABI)/libXnDeviceFile.so
LOCAL_SHARED_LIBRARIES := OpenNI XnCore XnFormats XnDDK

include $(PREBUILT_SHARED_LIBRARY)
#===========================================================

# XnDeviceSensorV2 =============================================
include $(CLEAR_VARS)
LOCAL_MODULE := XnDeviceSensorV2

LOCAL_SRC_FILES := lib/$(sensor_conf)/$(TARGET_ARCH_ABI)/libXnDeviceSensorV2.so
LOCAL_SHARED_LIBRARIES := OpenNI XnCore XnFormats XnDDK

include $(PREBUILT_SHARED_LIBRARY)
#===============================================================

# XnFormats =============================================
include $(CLEAR_VARS)
LOCAL_MODULE := XnFormats

LOCAL_SRC_FILES := lib/$(sensor_conf)/$(TARGET_ARCH_ABI)/libXnFormats.so
LOCAL_SHARED_LIBRARIES := OpenNI XnCore

include $(PREBUILT_SHARED_LIBRARY)
#===============================================================

$(call import-module,OpenNI)
