#!/bin/sh

type arm-linux-androideabi-gcc > /dev/null || {
  cat << EOF
Please create a standalone toolchain first and add it to your PATH. See
STANDALONE-TOOLCHAIN.html in Android NDK documentation.
EOF
  exit 1
}

(cd src/libusb/android && bash build.sh armeabi-v7a)

rm -r ndk-modules/libusb-1.0 2>/dev/null
cp -a src/libusb/android/ndk-modules/libusb-1.0 ndk-modules
