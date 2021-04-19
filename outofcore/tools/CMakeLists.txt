set(SUBSYS_NAME tools)

# pcl_outofcore_process
PCL_ADD_EXECUTABLE(pcl_outofcore_process COMPONENT ${SUBSYS_NAME} SOURCES outofcore_process.cpp)
target_link_libraries(pcl_outofcore_process pcl_common pcl_filters pcl_io pcl_octree pcl_outofcore)

PCL_ADD_EXECUTABLE(pcl_outofcore_print COMPONENT ${SUBSYS_NAME} SOURCES outofcore_print.cpp)
target_link_libraries(pcl_outofcore_print pcl_common pcl_filters pcl_io pcl_octree pcl_outofcore)

if(NOT VTK_FOUND)
  set(DEFAULT FALSE)
  set(REASON "VTK was not found.")
else()
  set(DEFAULT TRUE)
  set(REASON)
  include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")

  set(srcs outofcore_viewer.cpp
         ../src/visualization/camera.cpp
         ../src/visualization/common.cpp
         ../src/visualization/object.cpp
         ../src/visualization/grid.cpp
         ../src/visualization/outofcore_cloud.cpp
         ../src/visualization/scene.cpp
         ../src/visualization/viewport.cpp)

  # pcl_outofcore_viewer
  PCL_ADD_EXECUTABLE(pcl_outofcore_viewer COMPONENT ${SUBSYS_NAME} SOURCES ${srcs} BUNDLE)
  target_link_libraries(pcl_outofcore_viewer pcl_common pcl_io pcl_outofcore pcl_visualization pcl_octree pcl_filters)
endif()
