LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := onirec
LOCAL_SRC_FILES := ./com_pcl_onirec_Application.cpp ./com_pcl_onirec_CaptureThreadManager.cpp
LOCAL_LDLIBS := -ljnigraphics -llog

include $(BUILD_SHARED_LIBRARY)
