cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(concatenate_points)

find_package(PCL 1.2 REQUIRED)

add_executable (concatenate_points concatenate_points.cpp)
target_link_libraries (concatenate_points ${PCL_LIBRARIES})
