cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(pcl-matrix_transform)

find_package(PCL 1.7 REQUIRED)

add_executable (matrix_transform matrix_transform.cpp)
target_link_libraries (matrix_transform ${PCL_LIBRARIES})
