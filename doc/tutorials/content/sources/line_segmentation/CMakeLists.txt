cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(line_segmentation)

find_package(PCL 1.1 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (line_segmentation line_segmentation.cpp)
target_link_libraries (line_segmentation ${PCL_LIBRARIES})
