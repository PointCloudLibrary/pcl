cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(planar_segmentation)

find_package(PCL 1.0 REQUIRED COMPONENTS common io filters segmentation)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (planar_segmentation planar_segmentation.cpp)
target_link_libraries (planar_segmentation ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} 
                                           ${PCL_FILTERS_LIBRARIES} ${PCL_SEGMENTATION_LIBRARIES})
