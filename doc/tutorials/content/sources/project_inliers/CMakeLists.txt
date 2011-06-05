cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(project_inliers)

find_package(PCL 1.0 REQUIRED COMPONENTS common filters segmentation)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (project_inliers project_inliers.cpp)
target_link_libraries (project_inliers ${PCL_COMMON_LIBRARIES} ${PCL_FILTERS_LIBRARIES} ${PCL_SEGMENTATION_LIBRARIES})
