LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := onirec
LOCAL_SRC_FILES := org_pointclouds_onirec_Application.cpp \
                   org_pointclouds_onirec_CaptureThreadManager.cpp \
                   org_pointclouds_onirec_grab_NativeBuffer.cpp
LOCAL_LDLIBS := -ljnigraphics -llog

include $(BUILD_SHARED_LIBRARY)
