cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(pcl-interactive_icp)

find_package(PCL 1.5 REQUIRED)


add_executable (interactive_icp interactive_icp.cpp)
target_link_libraries (interactive_icp ${PCL_LIBRARIES})
