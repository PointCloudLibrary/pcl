#!/bin/sh

NDK_MODULE_PATH=$(cd ndk-modules; pwd)
export NDK_MODULE_PATH

rm -r "$NDK_MODULE_PATH"/OpenNI/include/* "$NDK_MODULE_PATH"/OpenNI/lib/armeabi-v7a/*

(cd OpenNI/Platform/Android; ndk-build)

cp -a OpenNI/Include/* "$NDK_MODULE_PATH/OpenNI/include"

for libname in OpenNI OpenNI.jni nimCodecs nimMockNodes nimRecorder; do
 cp OpenNI/Platform/Android/obj/local/armeabi-v7a/lib$libname.so "$NDK_MODULE_PATH/OpenNI/lib/armeabi-v7a"
done

(cd Sensor/Platform/Android; ndk-build)

for libname in XnCore XnDDK XnDeviceFile XnDeviceSensorV2 XnFormats; do
 cp Sensor/Platform/Android/obj/local/armeabi-v7a/lib$libname.so "$NDK_MODULE_PATH/Sensor/lib/armeabi-v7a"
done
