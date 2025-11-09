cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(openni_grabber)

find_package(PCL 1.2 REQUIRED)

add_executable (openni_grabber openni_grabber.cpp)
target_link_libraries (openni_grabber ${PCL_LIBRARIES})
