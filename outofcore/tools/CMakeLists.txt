PCL_ADD_EXECUTABLE(pcl_outofcore_process ${SUBSYS_NAME} outofcore_process.cpp)
target_link_libraries(pcl_outofcore_process pcl_common pcl_filters pcl_io pcl_outofcore)

if(NOT VTK_FOUND)
  set(DEFAULT FALSE)
  set(REASON "VTK was not found.")
else(NOT VTK_FOUND)
  set(DEFAULT TRUE)
  set(REASON)
  set(VTK_USE_FILE ${VTK_USE_FILE} CACHE INTERNAL "VTK_USE_FILE")
  include (${VTK_USE_FILE})
  include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

  #PCL_ADD_EXECUTABLE(pcl_outofcore_viewer ${SUBSYS_NAME} outofcore_viewer.cpp)
  #target_link_libraries(pcl_outofcore_viewer pcl_common pcl_io pcl_outofcore pcl_visualization)

  set(srcs outofcore_viewer.cpp)
  
  set(incs )
    
  PCL_ADD_INCLUDES(pcl_outofcore_viewer ${SUBSYS_NAME} ${incs})
  PCL_ADD_EXECUTABLE(pcl_outofcore_viewer ${SUBSYS_NAME} ${srcs})
  target_link_libraries(pcl_outofcore_viewer pcl_common pcl_io pcl_outofcore pcl_visualization)
  
endif(NOT VTK_FOUND)
