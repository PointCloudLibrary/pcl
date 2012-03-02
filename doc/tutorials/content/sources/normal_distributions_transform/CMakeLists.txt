cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(normal_distributions_transform)

FIND_PACKAGE(PCL 1.5 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(normal_distributions_transform normal_distributions_transform.cpp )
target_link_libraries (normal_distributions_transform ${PCL_LIBRARIES})
