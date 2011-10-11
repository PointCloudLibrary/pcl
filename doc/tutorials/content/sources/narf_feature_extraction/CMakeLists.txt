cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(narf_feature_extraction)

find_package(PCL 1.3 REQUIRED)

if(WXWIDGETS_FOUND)
  message("Found!!!:"${PCL_LIBRARIES})
  include_directories(${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})

  add_executable (narf_feature_extraction narf_feature_extraction.cpp)
  target_link_libraries (narf_feature_extraction ${PCL_LIBRARIES})
else(WXWIDGETS_FOUND)
  message("WxWidgets is required to build ${PROJECT_NAME}.")
endif(WXWIDGETS_FOUND)

