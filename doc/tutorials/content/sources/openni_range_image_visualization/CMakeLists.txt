cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(openni_range_image_visualization)

find_package(PCL 1.3 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
  
add_executable (openni_range_image_visualization openni_range_image_visualization.cpp)
target_link_libraries (openni_range_image_visualization ${PCL_LIBRARIES})

