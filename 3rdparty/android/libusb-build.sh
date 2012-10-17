#!/bin/sh

type arm-linux-androideabi-gcc > /dev/null || {
  cat << EOF
Please create a standalone toolchain first and add it to your PATH. See
STANDALONE-TOOLCHAIN.html in Android NDK documentation.

For example:
$  export CC="${ANDROID_NDK}/toolchains/arm-linux-androideabi-4.4.3/prebuilt/linux-x86/bin/arm-linux-androideabi-gcc --sysroot=$SYSROOT"

EOF
  exit 1
}

cd libusb/android
bash build.sh armeabi-v7a
cp -a ndk-modules/libusb-1.0 ../../ndk-modules
