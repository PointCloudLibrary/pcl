cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(boxClipper3D_example)

add_definitions(-std=c++17 -Wall -g)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package(PCL 1.10 REQUIRED)

include_directories( ${PCL_INCLUDE_DIRS} )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(extract_points example_boxClipper3D.cpp)
target_link_libraries(extract_points ${PCL_LIBRARIES})
