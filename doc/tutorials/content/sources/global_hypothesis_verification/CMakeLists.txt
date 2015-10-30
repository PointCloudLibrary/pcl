cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(global_hypothesis_verification)

#Pcl
find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (global_hypothesis_verification global_hypothesis_verification.cpp)
target_link_libraries (global_hypothesis_verification ${PCL_LIBRARIES})
 
