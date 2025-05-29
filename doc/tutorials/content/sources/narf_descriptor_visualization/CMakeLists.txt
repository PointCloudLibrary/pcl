cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(narf_descriptor_visualization)

find_package(PCL 1.3 REQUIRED)

add_executable (narf_descriptor_visualization narf_descriptor_visualization.cpp)
target_link_libraries (narf_descriptor_visualization ${PCL_LIBRARIES})
