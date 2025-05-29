cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(concatenate_clouds)

find_package(PCL 1.2 REQUIRED)

add_executable (concatenate_clouds concatenate_clouds.cpp)
target_link_libraries (concatenate_clouds ${PCL_LIBRARIES})
