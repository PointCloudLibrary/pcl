cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(narf_feature_extraction)

set(PCL_DIR "$ENV{PCL_ROOT}")
find_package(PCL 1.0 REQUIRED COMPONENTS io range_image visualization keypoints features)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (narf_feature_extraction narf_feature_extraction.cpp)
target_link_libraries (narf_feature_extraction ${PCL_IO_LIBRARY})
target_link_libraries (narf_feature_extraction ${PCL_RANGE_IMAGE_LIBRARY})
target_link_libraries (narf_feature_extraction ${PCL_VISUALIZATION_LIBRARY})
target_link_libraries (narf_feature_extraction ${PCL_KEYPOINTS_LIBRARY})
target_link_libraries (narf_feature_extraction ${PCL_FEATURES_LIBRARY})
