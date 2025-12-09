cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(implicit_shape_model)

find_package(PCL 1.5 REQUIRED)

set(CMAKE_BUILD_TYPE Release)

add_executable (implicit_shape_model implicit_shape_model.cpp)
target_link_libraries (implicit_shape_model ${PCL_LIBRARIES})
