#!/bin/bash
export NDK_MODULE_PATH=../../../3rdparty/libusb-android/libusb/android/ndk-modules:../../../3rdparty/openni/android/OpenNI/Platform/Android/jni:../../../3rdparty/openni/android/OpenNI/Platform/Android/jni/Modules:../../../3rdparty/openni/android/Sensor/Platform/Android/jni

ndk-build
android update project -p . -n ONIRecorder -l <OpenNI>/Wrappers/OpenNI.java
ant debug



