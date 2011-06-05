cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(template_alignment)

find_package(PCL 1.0 REQUIRED COMPONENTS common io filters kdtree features)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (template_alignment template_alignment.cpp)
target_link_libraries (template_alignment 
                       ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_FILTERS_LIBRARIES} 
                       ${PCL_KDTREE_LIBRARIES} ${PCL_FEATURES_LIBRARIES})
