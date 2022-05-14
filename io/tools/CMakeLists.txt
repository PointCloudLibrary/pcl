set(SUBSYS_NAME tools)

if(WITH_OPENNI)
  PCL_ADD_EXECUTABLE(pcl_openni_grabber_example COMPONENT ${SUBSYS_NAME} SOURCES openni_grabber_example.cpp)
  target_link_libraries(pcl_openni_grabber_example pcl_common pcl_io)

  PCL_ADD_EXECUTABLE(pcl_openni_grabber_depth_example COMPONENT ${SUBSYS_NAME} SOURCES openni_grabber_depth_example.cpp)
  target_link_libraries(pcl_openni_grabber_depth_example pcl_common pcl_io)


  PCL_ADD_EXECUTABLE(pcl_openni_pcd_recorder COMPONENT ${SUBSYS_NAME} SOURCES openni_pcd_recorder.cpp)
  target_link_libraries(pcl_openni_pcd_recorder pcl_common pcl_io Boost::date_time)
endif()

PCL_ADD_EXECUTABLE(pcl_pcd_convert_NaN_nan COMPONENT ${SUBSYS_NAME} SOURCES pcd_convert_NaN_nan.cpp)
PCL_ADD_EXECUTABLE(pcl_pcd_introduce_nan COMPONENT ${SUBSYS_NAME} SOURCES pcd_introduce_nan.cpp)
PCL_ADD_EXECUTABLE(pcl_convert_pcd_ascii_binary COMPONENT ${SUBSYS_NAME} SOURCES convert_pcd_ascii_binary.cpp)
if(VTK_FOUND)
  PCL_ADD_EXECUTABLE(pcl_converter COMPONENT ${SUBSYS_NAME} SOURCES converter.cpp)
  target_link_libraries(pcl_converter pcl_common pcl_io)
endif()
PCL_ADD_EXECUTABLE(pcl_hdl_grabber COMPONENT ${SUBSYS_NAME} SOURCES hdl_grabber_example.cpp)
target_link_libraries(pcl_convert_pcd_ascii_binary pcl_common pcl_io)
target_link_libraries(pcl_hdl_grabber pcl_common pcl_io)
target_link_libraries(pcl_pcd_introduce_nan pcl_common pcl_io)

#libply inherited tools
add_subdirectory(ply)
