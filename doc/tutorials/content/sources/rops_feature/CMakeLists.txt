cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(rops_feature)

find_package(PCL 1.8 REQUIRED)

add_executable (rops_feature rops_feature.cpp)
target_link_libraries (rops_feature ${PCL_LIBRARIES})

