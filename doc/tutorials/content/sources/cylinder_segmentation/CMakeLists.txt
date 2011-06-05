cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(cylinder_segmentation)

find_package(PCL 1.0 REQUIRED COMPONENTS common io filters features kdtree segmentation)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (cylinder_segmentation cylinder_segmentation.cpp)
target_link_libraries (cylinder_segmentation ${PCL_COMMON_LIBRARIES}  ${PCL_IO_LIBRARIES} 
                                             ${PCL_FEATURES_LIBRARIES} ${PCL_FILTERS_LIBRARIES} 
                                             ${PCL_KDTREE_LIBRARIES}  ${PCL_SEGMENTATION_LIBRARIES}   )


