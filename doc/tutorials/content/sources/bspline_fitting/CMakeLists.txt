cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(bspline_fitting)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (bspline_fitting bspline_fitting.cpp)
target_link_libraries (bspline_fitting ${PCL_LIBRARIES})
