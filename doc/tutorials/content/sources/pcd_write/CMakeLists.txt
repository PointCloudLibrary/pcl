cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(pcd_write)

find_package(PCL 1.2 REQUIRED)

add_executable (pcd_write pcd_write.cpp)
target_link_libraries (pcd_write ${PCL_LIBRARIES})
