cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(normal_estimation_using_integral_images)

find_package(PCL 1.3 REQUIRED)

add_executable (normal_estimation_using_integral_images normal_estimation_using_integral_images.cpp)
target_link_libraries (normal_estimation_using_integral_images ${PCL_LIBRARIES})

