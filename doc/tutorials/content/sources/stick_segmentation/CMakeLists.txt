cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(stick_segmentation)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (stick_segmentation stick_segmentation.cpp)
target_link_libraries (stick_segmentation ${PCL_LIBRARIES})
