cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(pcl_visualizer_viewports)

find_package(PCL 1.0 REQUIRED COMPONENTS features visualization)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable (pcl_visualizer_normals pcl_visualizer_normals.cpp)
target_link_libraries (pcl_visualizer_normals ${PCL_LIBRARIES})
add_executable (pcl_visualizer_color_custom pcl_visualizer_color_custom.cpp)
target_link_libraries (pcl_visualizer_color_custom ${PCL_LIBRARIES})

add_executable (pcl_visualizer_color_rgb pcl_visualizer_color_rgb.cpp)
target_link_libraries (pcl_visualizer_color_rgb ${PCL_LIBRARIES})

add_executable (pcl_visualizer_viewports pcl_visualizer_viewports.cpp)
target_link_libraries (pcl_visualizer_viewports ${PCL_LIBRARIES})

add_executable (pcl_visualizer_shapes pcl_visualizer_shapes.cpp)
target_link_libraries (pcl_visualizer_shapes ${PCL_LIBRARIES})

add_executable (pcl_visualizer_simple pcl_visualizer_simple.cpp)
target_link_libraries (pcl_visualizer_simple ${PCL_LIBRARIES})

