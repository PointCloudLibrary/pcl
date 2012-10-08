cmake_minimum_required(VERSION 2.8)

project(HelloPCLWorld)

#notice the "recycling" of CMAKE_C_FLAGS
#this is necessary to pick up android flags
set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -pedantic" )




find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

find_package(VES REQUIRED)
include_directories(${VES_INCLUDE_DIRS})

include_directories(/home/andrey/kiwi/VES/src/ves)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/jni)

 

message(${VES_LIBRARIES})
	

set(deps
  GLESv2
  EGL
  android
  log
  ${VES_LIBRARIES}
)



find_package(Boost 1.45.0 REQUIRED COMPONENTS date_time system filesystem thread iostreams)
include_directories(${Boost_INCLUDE_DIRS})
find_package(PCL 1.3 REQUIRED COMPONENTS common io octree kdtree)
include_directories(/home/andrey/kiwi/pcl-superbuild/build/CMakeExternals/Install/pcl-android/include
/home/andrey/kiwi/pcl-superbuild/build/CMakeExternals/Install/pcl-android/include/pcl-1.6
/home/andrey/kiwi/VES/Apps/Android/CMakeBuild/build/CMakeExternals/Install/eigen 
/home/andrey/kiwi/VES/Apps/Android/CMakeBuild/build/CMakeExternals/Source/eigen ${PCL_INCLUDE_DIRS})  







#IMPORTANT
link_directories (/home/andrey/android-ndk-r8b/toolchains/arm-linux-androideabi-4.6/prebuilt/linux-x86/user/lib)
link_directories (/home/andrey/kiwi/pcl-superbuild/build/CMakeExternals/Build/pcl-android/lib)

#test for ANDROID and look for stuff not relevant to android.
if(NOT ANDROID)
#jni is available by default on android
find_package(JNI REQUIRED)
include_directories(${JNI_INCLUDE_DIRS})
endif()


#this will be overridden by the android toolchain
set( LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/lib/pc CACHE PATH
        "Single Directory for all Libraries")



#make sure to used shared in in the jni library, as this will be loaded by
#java

add_library(KiwiNative SHARED jni/KiwiNative.cpp jni/KiwiNative.cpp jni/myFunctions.cpp jni/profiler.cpp jni/pcl_demo.cpp)
target_link_libraries(KiwiNative ${BOOST_LIBRARIES}
/home/andrey/android-ndk-r8b/toolchains/arm-linux-androideabi-4.6/prebuilt/linux-x86/user/lib/libboost_filesystem.a
log
pcl_kdtree
pcl_io
pcl_features
pcl_search
flann_cpp_s pcl_common ${deps})


