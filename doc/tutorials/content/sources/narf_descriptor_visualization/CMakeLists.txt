cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(narf_descriptor_visualization)

find_package(PCL 1.3 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (narf_descriptor_visualization narf_descriptor_visualization.cpp)
target_link_libraries (narf_descriptor_visualization ${PCL_LIBRARIES})
