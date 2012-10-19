#!/bin/sh

export NDK_MODULE_PATH=$(cd ../../../3rdparty/android/ndk-modules; pwd)
ndk-build
android update project -p . -n BodyParts  
ant debug
