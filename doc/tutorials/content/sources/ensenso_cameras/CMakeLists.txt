cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pcl-ensenso_cloud_images_viewer)

find_package(PCL 1.8.0 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(ensenso_cloud_images_viewer ensenso_cloud_images_viewer.cpp)
target_link_libraries(ensenso_cloud_images_viewer ${PCL_LIBRARIES} ${OpenCV_LIBS})
