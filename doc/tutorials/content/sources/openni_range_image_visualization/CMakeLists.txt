cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(openni_range_image_visualization)

find_package(PCL 1.3 REQUIRED)

add_executable (openni_range_image_visualization openni_range_image_visualization.cpp)
target_link_libraries (openni_range_image_visualization ${PCL_LIBRARIES})
