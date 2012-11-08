#!/bin/sh

NDK_MODULE_PATH=$(cd ndk-modules; pwd)
export NDK_MODULE_PATH

rm -r "$NDK_MODULE_PATH"/OpenNI/include/* "$NDK_MODULE_PATH"/OpenNI/lib/armeabi-v7a/* 2>/dev/null

cp -a src/OpenNI/Include/* "$NDK_MODULE_PATH/OpenNI/include"

(cd src/OpenNI/Platform/Android; ndk-build -j2)

for libname in OpenNI OpenNI.jni nimCodecs nimMockNodes nimRecorder; do
 cp src/OpenNI/Platform/Android/obj/local/armeabi-v7a/lib$libname.so "$NDK_MODULE_PATH/OpenNI/lib/armeabi-v7a"
done

rm -r "$NDK_MODULE_PATH"/Sensor/lib/armeabi-v7a/* 2>/dev/null

(cd src/Sensor/Platform/Android; ndk-build -j2)

for libname in XnCore XnDDK XnDeviceFile XnDeviceSensorV2 XnFormats; do
 cp src/Sensor/Platform/Android/obj/local/armeabi-v7a/lib$libname.so "$NDK_MODULE_PATH/Sensor/lib/armeabi-v7a"
done
