cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(function_filter)

find_package(PCL 1.11.1.99 REQUIRED)

add_executable (sphere_removal sphere_removal.cpp)
target_link_libraries (sphere_removal ${PCL_LIBRARIES})
add_definitions(${PCL_DEFINITIONS})
