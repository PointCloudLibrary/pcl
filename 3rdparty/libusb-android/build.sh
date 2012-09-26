#!/bin/sh

type arm-linux-androideabi-gcc > /dev/null || {
  cat << EOF
Please create a standalone toolchain first and add it to your PATH. See
STANDALONE-TOOLCHAIN.html in Android NDK documentation.
EOF
  exit 1
}

cd libusb/android
bash build.sh armeabi
bash build.sh armeabi-v7a
