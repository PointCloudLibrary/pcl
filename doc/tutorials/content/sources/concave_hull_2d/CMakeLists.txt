cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(concave_hull_2d)

find_package(PCL 1.2 REQUIRED)

add_executable (concave_hull_2d concave_hull_2d.cpp)
target_link_libraries (concave_hull_2d ${PCL_LIBRARIES})
