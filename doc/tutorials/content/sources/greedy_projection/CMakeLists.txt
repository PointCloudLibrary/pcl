cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(greedy_projection)

find_package(PCL 1.0 REQUIRED COMPONENTS common io kdtree features surface)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (greedy_projection greedy_projection.cpp)
target_link_libraries (greedy_projection ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} 
                                         ${PCL_KDTREE_LIBRARIES} ${PCL_SURFACE_LIBRARIES}
                                         ${PCL_FEATURES_LIBRARIES})
