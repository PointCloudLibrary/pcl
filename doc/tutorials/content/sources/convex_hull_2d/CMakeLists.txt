cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(convex_hull_2d)

find_package(PCL 1.0 REQUIRED COMPONENTS common io filters segmentation surface)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (convex_hull_2d convex_hull_2d.cpp)
target_link_libraries (convex_hull_2d ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} 
                                      ${PCL_FILTERS_LIBRARIES} ${PCL_SEGMENTATION_LIBRARIES}  
                                      ${PCL_SURFACE_LIBRARIES})
