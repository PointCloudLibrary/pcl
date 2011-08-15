cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(openni_narf_keypoint_extraction)

find_package(PCL 1.1 REQUIRED)

if(WXWIDGETS_FOUND)
  include_directories(${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})
  
  add_executable (openni_narf_keypoint_extraction openni_narf_keypoint_extraction.cpp)
  target_link_libraries (openni_narf_keypoint_extraction ${PCL_IO_LIBRARY})
  target_link_libraries (openni_narf_keypoint_extraction ${PCL_RANGE_IMAGE_LIBRARY})
  target_link_libraries (openni_narf_keypoint_extraction ${PCL_VISUALIZATION_LIBRARY})
  target_link_libraries (openni_narf_keypoint_extraction ${PCL_KEYPOINTS_LIBRARY})
else(WXWIDGETS_FOUND)
  message("WxWidgets is required to build ${PROJECT_NAME}.")
endif(WXWIDGETS_FOUND)

