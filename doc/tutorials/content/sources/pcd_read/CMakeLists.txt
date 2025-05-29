cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(pcd_read)

find_package(PCL 1.2 REQUIRED)

add_executable (pcd_read pcd_read.cpp)
target_link_libraries (pcd_read ${PCL_LIBRARIES})
