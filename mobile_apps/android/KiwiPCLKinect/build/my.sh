ANT=`which ant`
ADB=`which adb`
ANDROID=`which android`
CMAKE=`which cmake`
export ANDROID_NDK=/home/andrey/android-ndk-r8b

cmakeexternals=/home/andrey/kiwi/VES/Apps/Android/CMakeBuild/build/CMakeExternals
pcl_dir=/home/andrey/kiwi/pcl-superbuild/build/CMakeExternals/Install/pcl-android
flann_dir=/home/andrey/kiwi/pcl-superbuild/build/CMakeExternals/Build/flann-android

TOOLCHAIN=/home/andrey/kiwi/pcl-superbuild/build/CMakeExternals/Source/pcl/mobile_apps/android/android_dependencies/android-cmake/toolchain/android.toolchain.cmake

VTK_DIR=$cmakeexternals/Build/vtk-android
VES_DIR=$cmakeexternals/Build/ves-android

app_dir=$(cd $(dirname $0)/.. && pwd)
source_dir=/home/andrey/kiwi/VES
build_dir=$app_dir

pcl_include=$pcl_dir/include/pcl-1.6
pcl_lib=$pcl_dir/lib

set -x

mkdir -p $build_dir
cd $build_dir
cd build
pwd
#android-cmake .
cmake \
    -DCMAKE_BUILD_TYPE=Debug                                     \
    -DCMAKE_TOOLCHAIN_FILE=$TOOLCHAIN				 \
    -DCMAKE_INSTALL_NAME_TOOL=/usr/bin/install_name_tool         \
    -DANDROID_NATIVE_API_LEVEL=14                                \
    -DVTK_DIR=$VTK_DIR                                           \
    -DVES_DIR=$VES_DIR $app_dir                                  \
    -DPCL_DIR=$pcl_dir/share/pcl-1.6  				 \
    -DPCL_LIBRARIES=$pcl_dir/lib       				 \
    -DFLANN_DIR=$flann_dir             				 \
								 \
    -DPCL_COMMON_INCLUDE_DIR=$pcl_include 			 \
    -DPCL_IO_INCLUDE_DIR=$pcl_include 			 	 \
    -DPCL_OCTREE_INCLUDE_DIR=$pcl_include 			 \
    -DPCL_KDTREE_INCLUDE_DIR=$pcl_include 			 \
    -DPCL_FEATURES_INCLUDE_DIR=$pcl_include 			 \
    -DPCL_SEARCH_INCLUDE_DIR=$pcl_include 			 \
								 \
    -DPCL_COMMON_LIBRARY=$pcl_lib/libpcl_common.a 		 \
    -DPCL_IO_LIBRARY=$pcl_lib/libpcl_io.a 			 \
    -DPCL_OCTREE_LIBRARY=$pcl_lib/libpcl_octree.a 		 \
    -DPCL_KDTREE_LIBRARY=$pcl_lib/libpcl_kdtree.a 		 \
    -DPCL_FEATURES_LIBRARY=$pcl_lib/libpcl_features.a 		 \
    -DPCL_SEARCH_LIBRARY=$pcl_lib/libpcl_search.a ..	

