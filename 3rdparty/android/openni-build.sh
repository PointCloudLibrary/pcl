#!/bin/sh

NDK_MODULE_PATH=$(cd ndk-modules; pwd)
export NDK_MODULE_PATH

rm -r "$NDK_MODULE_PATH"/OpenNI/include/* \
      "$NDK_MODULE_PATH"/OpenNI/lib/debug/armeabi-v7a/* \
      "$NDK_MODULE_PATH"/OpenNI/lib/release/armeabi-v7a/* 2>/dev/null

cp -a src/OpenNI/Include/* "$NDK_MODULE_PATH/OpenNI/include"

openni_libs="OpenNI OpenNI.jni nimCodecs nimMockNodes nimRecorder"

rm src/OpenNI/Platform/Android/obj/local/armeabi-v7a/lib*.so 2>/dev/null

(cd src/OpenNI/Platform/Android; ndk-build -j2)

for libname in $openni_libs; do
 cp src/OpenNI/Platform/Android/obj/local/armeabi-v7a/lib$libname.so \
   "$NDK_MODULE_PATH/OpenNI/lib/release/armeabi-v7a"
done

rm src/OpenNI/Platform/Android/obj/local/armeabi-v7a/lib*.so

(cd src/OpenNI/Platform/Android; ndk-build -j2 NDK_DEBUG=1)

for libname in $openni_libs; do
 cp src/OpenNI/Platform/Android/obj/local/armeabi-v7a/lib$libname.so \
   "$NDK_MODULE_PATH/OpenNI/lib/debug/armeabi-v7a"
done

rm -r "$NDK_MODULE_PATH"/Sensor/lib/debug/armeabi-v7a/* \
      "$NDK_MODULE_PATH"/Sensor/lib/release/armeabi-v7a/* 2>/dev/null
      
sensor_libs="XnCore XnDDK XnDeviceFile XnDeviceSensorV2 XnFormats"

rm src/Sensor/Platform/Android/obj/local/armeabi-v7a/lib*.so 2>/dev/null

(cd src/Sensor/Platform/Android; ndk-build -j2)

for libname in $sensor_libs; do
 cp src/Sensor/Platform/Android/obj/local/armeabi-v7a/lib$libname.so \
   "$NDK_MODULE_PATH/Sensor/lib/release/armeabi-v7a"
done

rm src/Sensor/Platform/Android/obj/local/armeabi-v7a/lib*.so

(cd src/Sensor/Platform/Android; ndk-build -j2 NDK_DEBUG=1)

for libname in $sensor_libs; do
 cp src/Sensor/Platform/Android/obj/local/armeabi-v7a/lib$libname.so \
   "$NDK_MODULE_PATH/Sensor/lib/debug/armeabi-v7a"
done
