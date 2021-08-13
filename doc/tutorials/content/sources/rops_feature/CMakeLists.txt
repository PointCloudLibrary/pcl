cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(rops_feature)

find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (rops_feature rops_feature.cpp)
target_link_libraries (rops_feature ${PCL_LIBRARIES})

