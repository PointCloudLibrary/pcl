cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(project_inliers)

find_package(PCL 1.2 REQUIRED)

add_executable (project_inliers project_inliers.cpp)
target_link_libraries (project_inliers ${PCL_LIBRARIES})
