cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(pcl-ensenso_cloud_images_viewer)

find_package(PCL 1.8.0 REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(ensenso_cloud_images_viewer ensenso_cloud_images_viewer.cpp)
target_link_libraries(ensenso_cloud_images_viewer ${PCL_LIBRARIES} ${OpenCV_LIBS})
