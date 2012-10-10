# download the build scripts
git clone git://github.com/patmarion/pcl-superbuild.git ../../pcl-superbuild

# configure; toggle the options BUILD_ANDROID and BUILD_IOS_DEVICE as you like
cd ../../pcl-superbuild/ && mkdir -p build && cd build && cmake -DBUILD_IOS_DEVICE:BOOL="0" ../

# build 
export ANDROID_NDK=${NDKROOT}
cmake -DBUILD_examples:BOOL="0"  ../../pcl-superbuild/build/CMakeExternals/Build/pcl-android/
cd ../../pcl-superbuild/build/ && make
