#!/bin/bash
#shall run the permission for the first time only  (see fix_sd_permission.sh)
#adb shell mount -o remount rw /system
#adb push platform.xml /system/etc/permissions/
#this will get created automatically through the apps
#adb shell mkdir /mnt/sdcard2/pcl
adb shell mount -o devmode=0666 -t usbfs none /proc/bus/usb
adb shell am force-stop com.pcl.opennirecorder
adb shell am start -n com.pcl.opennirecorder/android.app.NativeActivity
