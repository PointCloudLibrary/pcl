cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(narf_feature_extraction)

set(PCL_DIR "$ENV{PCL_ROOT}")
find_package(PCL 1.0 REQUIRED COMPONENTS io range_image visualization keypoints features)

if(WXWIDGETS_FOUND)
  include_directories(${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})

  add_executable (narf_feature_extraction narf_feature_extraction.cpp)
  target_link_libraries (narf_feature_extraction ${PCL_IO_LIBRARIES})
  target_link_libraries (narf_feature_extraction ${PCL_RANGE_IMAGE_LIBRARIES})
  target_link_libraries (narf_feature_extraction ${PCL_VISUALIZATION_LIBRARIES})
  target_link_libraries (narf_feature_extraction ${PCL_KEYPOINTS_LIBRARIES})
  target_link_libraries (narf_feature_extraction ${PCL_FEATURES_LIBRARIES})
else(WXWIDGETS_FOUND)
  message("WxWidgets is required to build ${PROJECT_NAME}.")
endif(WXWIDGETS_FOUND)

