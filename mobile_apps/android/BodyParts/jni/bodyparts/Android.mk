LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := bodyparts
LOCAL_SRC_FILES := body_parts_recognizer.cpp openni_grabber.cpp com_itseez_peopledemo_BodyPartsRecognizer.cpp \
  file_grabber.cpp rgbd_image.cpp com_itseez_peopledemo_Grabber.cpp stopwatch.cpp \
  com_itseez_peopledemo_RGBDImage.cpp
LOCAL_LDLIBS := -llog
LOCAL_STATIC_LIBRARIES := boost boost_filesystem tbb
LOCAL_SHARED_LIBRARIES := OpenNI

include $(BUILD_SHARED_LIBRARY)
