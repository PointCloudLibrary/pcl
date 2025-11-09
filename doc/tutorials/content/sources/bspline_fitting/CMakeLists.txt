cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(bspline_fitting)

find_package(PCL 1.7 REQUIRED)

add_executable (bspline_fitting bspline_fitting.cpp)
target_link_libraries (bspline_fitting ${PCL_LIBRARIES})
