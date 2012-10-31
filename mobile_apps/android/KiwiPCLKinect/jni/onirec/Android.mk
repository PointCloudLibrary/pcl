LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := onirec
LOCAL_SRC_FILES := ./com_itseez_icpdemo_Application.cpp
LOCAL_LDLIBS := -ljnigraphics -llog

include $(BUILD_SHARED_LIBRARY)
