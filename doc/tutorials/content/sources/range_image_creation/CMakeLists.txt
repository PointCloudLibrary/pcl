cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(range_image_creation)

find_package(PCL 1.2 REQUIRED)

add_executable (range_image_creation range_image_creation.cpp)
target_link_libraries (range_image_creation ${PCL_LIBRARIES})
