cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(correspondence_grouping)

find_package(PCL 1.5 REQUIRED)

add_executable (correspondence_grouping correspondence_grouping.cpp)
target_link_libraries (correspondence_grouping ${PCL_LIBRARIES})
