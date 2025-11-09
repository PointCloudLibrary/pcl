cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(bare_earth)

find_package(PCL 1.7.2 REQUIRED)

add_executable (bare_earth bare_earth.cpp)
target_link_libraries (bare_earth ${PCL_LIBRARIES})
