adb shell mount -o remount rw /system
adb push platform.xml /system/etc/permissions/
adb shell reboot
