#!/bin/bash
export NDK_MODULE_PATH=$(cd ../../../3rdparty/android/ndk-modules; pwd)

ndk-build
android update project -p . -n ONIRecorder \
  -l ../../../3rdparty/android/OpenNI/Wrappers/OpenNI.java \
  -l ../../../3rdparty/android/libusb/android/LibusbSupport
ant debug
