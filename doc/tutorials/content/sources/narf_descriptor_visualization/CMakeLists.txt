cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(narf_descriptor_visualization)

set(PCL_DIR "$ENV{PCL_ROOT}")
find_package(PCL 1.0 REQUIRED COMPONENTS io range_image visualization features)

if(WXWIDGETS_FOUND)
  include_directories(${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})

  add_executable (narf_descriptor_visualization narf_descriptor_visualization.cpp)
  target_link_libraries (narf_descriptor_visualization ${PCL_IO_LIBRARIES})
  target_link_libraries (narf_descriptor_visualization ${PCL_RANGE_IMAGE_LIBRARIES})
  target_link_libraries (narf_descriptor_visualization ${PCL_VISUALIZATION_LIBRARIES})
  target_link_libraries (narf_descriptor_visualization ${PCL_FEATURES_LIBRARIES})
else(WXWIDGETS_FOUND)
  message("WxWidgets is required to build ${PROJECT_NAME}.")
endif(WXWIDGETS_FOUND)

