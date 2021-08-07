cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(point_cloud_compression)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (point_cloud_compression point_cloud_compression.cpp)
target_link_libraries (point_cloud_compression ${PCL_LIBRARIES})
