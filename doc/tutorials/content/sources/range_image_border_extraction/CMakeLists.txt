cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(range_image_border_extraction)

find_package(PCL 1.1 REQUIRED)

if(WXWIDGETS_FOUND)
  include_directories(${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})

  add_executable (range_image_border_extraction range_image_border_extraction.cpp)
  target_link_libraries (range_image_border_extraction ${PCL_RANGE_IMAGE_LIBRARIES})
  target_link_libraries (range_image_border_extraction ${PCL_VISUALIZATION_LIBRARIES})
  target_link_libraries (range_image_border_extraction ${PCL_KEYPOINTS_LIBRARIES})
else(WXWIDGETS_FOUND)
  message("WxWidgets is required to build ${PROJECT_NAME}.")
endif(WXWIDGETS_FOUND)

