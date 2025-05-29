cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(global_hypothesis_verification)

#Pcl
find_package(PCL 1.7 REQUIRED)

add_executable (global_hypothesis_verification global_hypothesis_verification.cpp)
target_link_libraries (global_hypothesis_verification ${PCL_LIBRARIES})
