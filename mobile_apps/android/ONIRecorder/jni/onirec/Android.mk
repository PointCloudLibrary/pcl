LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := onirec
LOCAL_SRC_FILES := com_itseez_onirec_Application.cpp \
                   com_itseez_onirec_CaptureThreadManager.cpp \
                   com_itseez_onirec_grab_NativeBuffer.cpp
LOCAL_LDLIBS := -ljnigraphics -llog

include $(BUILD_SHARED_LIBRARY)
