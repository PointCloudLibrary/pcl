if(OPENNI_FOUND)
  PCL_ADD_EXECUTABLE(pcl_openni_grabber_example ${SUBSYS_NAME} openni_grabber_example.cpp)
  target_link_libraries(pcl_openni_grabber_example pcl_common pcl_io)

  PCL_ADD_EXECUTABLE(pcl_openni_grabber_depth_example ${SUBSYS_NAME} openni_grabber_depth_example.cpp)
  target_link_libraries(pcl_openni_grabber_depth_example pcl_common pcl_io)


  PCL_ADD_EXECUTABLE(pcl_openni_pcd_recorder ${SUBSYS_NAME} openni_pcd_recorder.cpp)
  target_link_libraries(pcl_openni_pcd_recorder pcl_common pcl_io)

endif(OPENNI_FOUND)

PCL_ADD_EXECUTABLE(pcl_pcd_convert_NaN_nan ${SUBSYS_NAME} pcd_convert_NaN_nan.cpp)
PCL_ADD_EXECUTABLE(pcl_convert_pcd_ascii_binary ${SUBSYS_NAME} convert_pcd_ascii_binary.cpp)
PCL_ADD_EXECUTABLE(pcl_hdl_grabber ${SUBSYS_NAME} hdl_grabber_example.cpp)
target_link_libraries(pcl_convert_pcd_ascii_binary pcl_common pcl_io)
target_link_libraries(pcl_hdl_grabber pcl_common pcl_io)

#libply inherited tools
add_subdirectory(ply)
