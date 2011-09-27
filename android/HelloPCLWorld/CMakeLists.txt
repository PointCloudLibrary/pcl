cmake_minimum_required(VERSION 2.8)

project(HelloPCLWorld)

#notice the "recycling" of CMAKE_C_FLAGS
#this is necessary to pick up android flags
#set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -pedantic" )

find_package(Boost REQUIRED filesystem system)
find_package(PCL 1.2 REQUIRED COMPONENTS common io)
include_directories(${PCL_INCLUDE_DIRS})  

include_directories(${Boost_INCLUDE_DIRS})


#make sure to used shared in in the jni library, as this will be loaded by
#java

add_executable(helloPCLWorld hello-pcl.cpp)
target_link_libraries(helloPCLWorld ${Boost_LIBRARIES} log pcl_io pcl_common)