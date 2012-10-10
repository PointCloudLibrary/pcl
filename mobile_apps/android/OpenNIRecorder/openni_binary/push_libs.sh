#adb shell
adb shell mkdir /data/ni
adb shell mount -o remount rw /system
adb shell mount -o devmode=0666 -t usbfs none /proc/bus/usb

#old binaries
#adb push system/lib/libOpenNI.so /system/lib
#adb push system/lib/libOpenNI.jni.so /system/lib
#adb push system/lib/libXnCore.so  /system/lib
#adb push system/lib/libXnDDK.so /system/lib
#adb push system/lib/libXnDeviceFile.so /system/lib
#adb push system/lib/libXnDeviceSensorV2.so /system/lib
#adb push system/lib/libXnFormats.so /system/lib
#adb push system/lib/libusb.so /system/lib
#adb push system/lib/libnimCodecs.so /system/lib
#adb push system/lib/libnimRecorder.so /system/lib
#adb push system/lib/libnimMockNodes.so /system/lib
#adb push system/lib/libXnVNite_1_5_0.so /system/lib

adb shell mount -o remount rw /system
adb push ../../openni_drivers/openni/build/openni/Platform/Android/libs/armeabi-v7a/libnimCodecs.so /system/lib
adb push ../../openni_drivers/openni/build/openni/Platform/Android/libs/armeabi-v7a/libnimMockNodes.so /system/lib
adb push ../../openni_drivers/openni/build/openni/Platform/Android/libs/armeabi-v7a/libnimRecorder.so /system/lib
adb push ../../openni_drivers/openni/build/openni/Platform/Android/libs/armeabi-v7a/libOpenNI.jni.so /system/lib
adb push ../../openni_drivers/openni/build/openni/Platform/Android/libs/armeabi-v7a/libOpenNI.so /system/lib
adb push ../../openni_drivers/ps_engine/build/ps_engine/Platform/Android/libs/armeabi-v7a/libOpenNI.so /system/lib
adb push ../../openni_drivers/ps_engine/build/ps_engine/Platform/Android/libs/armeabi-v7a/libusb.so /system/lib
adb push ../../openni_drivers/ps_engine/build/ps_engine/Platform/Android/libs/armeabi-v7a/libXnCore.so /system/lib
adb push ../../openni_drivers/ps_engine/build/ps_engine/Platform/Android/libs/armeabi-v7a/libXnDDK.so /system/lib
adb push ../../openni_drivers/ps_engine/build/ps_engine/Platform/Android/libs/armeabi-v7a/libXnDeviceFile.so /system/lib
adb push ../../openni_drivers/ps_engine/build/ps_engine/Platform/Android/libs/armeabi-v7a/libXnDeviceSensorV2.so /system/lib
adb push ../../openni_drivers/ps_engine/build/ps_engine/Platform/Android/libs/armeabi-v7a/libXnFormats.so /system/lib
adb push modules.xml /data/ni/
adb push GlobalDefaults.ini /data/ni/
adb push SamplesConfig.xml /data/ni/

#optional
