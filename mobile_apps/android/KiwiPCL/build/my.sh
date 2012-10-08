ANT=`which ant`
ADB=`which adb`
ANDROID=`which android`
CMAKE=`which cmake`
export ANDROID_NDK=/home/andrey/android-ndk-r8b

app_dir=$(cd $(dirname $0)/.. && pwd)
source_dir=/home/andrey/kiwi/VES
build_dir=$app_dir

cmakeexternals=/home/andrey/kiwi/VES/Apps/Android/CMakeBuild/build/CMakeExternals

set -x

VTK_DIR=$cmakeexternals/Build/vtk-android
VES_DIR=$cmakeexternals/Build/ves-android
TOOLCHAIN=$source_dir/CMake/toolchains/android.toolchain.cmake

mkdir -p $build_dir
cd $build_dir
cd build
pwd
#android-cmake .
cmake -DCMAKE_TOOLCHAIN_FILE=/home/andrey/kiwi/pcl-superbuild/build/CMakeExternals/Source/pcl/mobile_apps/android/android_dependencies/android-cmake/toolchain/android.toolchain.cmake -DCMAKE_INSTALL_NAME_TOOL=/usr/bin/install_name_tool -DANDROID_NATIVE_API_LEVEL=14 -DVTK_DIR=$VTK_DIR -DVES_DIR=$VES_DIR $app_dir -DPCL_DIR=/home/andrey/kiwi/pcl-superbuild/build/CMakeExternals/Install/pcl-android -DPCL_LIBRARIES=/home/andrey/kiwi/pcl-superbuild/build/CMakeExternals/Build/pcl-android ..
#$CMAKE -DCMAKE_TOOLCHAIN_FILE=$TOOLCHAIN -DANDROID_NATIVE_API_LEVEL=14 -DVTK_DIR=$VTK_DIR -DVES_DIR=$VES_DIR $app_dir
