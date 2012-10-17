#!/bin/sh

cd tbb41_20120718oss

make ANDROID=1

rm -r ../ndk-mobules/tbb/include/* 2> /dev/null
cp -a include/tbb ../ndk-modules/tbb/include

rm -r ../ndk-modules/tbb/lib/armeabi-v7a/* 2> /dev/null
cp build/android_arm_gcc_whatever_release/*.so ../ndk-modules/tbb/lib/armeabi-v7a
