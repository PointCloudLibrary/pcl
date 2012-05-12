cmake_minimum_required(VERSION 2.8)

project(HelloPCLWorld)

#notice the "recycling" of CMAKE_C_FLAGS
#this is necessary to pick up android flags
set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -pedantic" )


find_package(Boost 1.45.0 REQUIRED COMPONENTS date_time system filesystem thread iostreams)
include_directories(${Boost_INCLUDE_DIRS})
find_package(PCL 1.3 REQUIRED COMPONENTS common io octree kdtree)
include_directories(${PCL_INCLUDE_DIRS})  

#IMPORTANT
link_directories (/Users/alex/Work/Android/android-ndk-r6b/toolchains/arm-linux-androideabi-4.4.3/prebuilt/darwin-x86/user/lib/)

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

add_library(helloPCLWorld SHARED jni/hello-pcl.cpp)
target_link_libraries(helloPCLWorld ${BOOST_LIBRARIES}
/Users/alex/Work/Android/android-ndk-r6b/toolchains/arm-linux-androideabi-4.4.3/prebuilt/darwin-x86/user/lib/libboost_filesystem.a
log
pcl_kdtree
pcl_io
flann_cpp_s pcl_common)


