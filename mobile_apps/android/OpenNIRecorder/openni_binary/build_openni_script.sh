#!/bin/bash


#make sure ../../ is writeable
#hg clone https://kforge.ros.org/openni/drivers ../../openni_drivers
hg clone https://kforge.ros.org/openni/drivers -r 12111edd3b15 ../../openni_drivers
cd ../../openni_drivers/openni && make && cd -
rm -rf ../../openni_drivers/openni/build/openni/Platform/Android/jni/Samples/NiSkeletonBenchmark
unset NDK_MODULE_PATH
cd ../../openni_drivers/openni/build/openni/Platform/Android/jni && ${NDKROOT}/ndk-build && cd -

cd ../../openni_drivers/ps_engine && make && cd -
export NDK_MODULE_PATH=`pwd`/../../openni_drivers/openni/build/openni/Platform/Android/jni
cd ../../openni_drivers/ps_engine/build/ps_engine/Platform/Android/jni && ${NDKROOT}/ndk-build && cd -
