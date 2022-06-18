cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(region_growing_rgb_segmentation)

find_package(PCL 1.5 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (region_growing_rgb_segmentation region_growing_rgb_segmentation.cpp)
target_link_libraries (region_growing_rgb_segmentation ${PCL_LIBRARIES})


