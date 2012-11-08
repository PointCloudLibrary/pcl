#!/bin/sh

(cd src/tbb && make ANDROID=1)

rm -r ndk-mobules/tbb/include/* 2> /dev/null
rm -r ndk-modules/tbb/lib/armeabi-v7a/* 2> /dev/null

cp -a src/tbb/include/tbb ndk-modules/tbb/include
cp src/tbb/build/android_arm_gcc_whatever_release/*.so ndk-modules/tbb/lib/armeabi-v7a
