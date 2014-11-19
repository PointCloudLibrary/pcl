cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(openni_tracking)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (tracking_sample tracking_sample.cpp)
target_link_libraries (tracking_sample ${PCL_LIBRARIES})
