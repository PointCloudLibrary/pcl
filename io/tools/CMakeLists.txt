if(WITH_OPENNI)
  PCL_ADD_EXECUTABLE(pcl_openni_grabber_example ${SUBSYS_NAME} openni_grabber_example.cpp)

  PCL_ADD_EXECUTABLE(pcl_openni_grabber_depth_example ${SUBSYS_NAME} openni_grabber_depth_example.cpp)


  PCL_ADD_EXECUTABLE(pcl_openni_pcd_recorder ${SUBSYS_NAME} openni_pcd_recorder.cpp)
endif()

PCL_ADD_EXECUTABLE(pcl_pcd_convert_NaN_nan ${SUBSYS_NAME} pcd_convert_NaN_nan.cpp)
PCL_ADD_EXECUTABLE(pcl_pcd_introduce_nan ${SUBSYS_NAME} pcd_introduce_nan.cpp)
PCL_ADD_EXECUTABLE(pcl_convert_pcd_ascii_binary ${SUBSYS_NAME} convert_pcd_ascii_binary.cpp)
if(VTK_FOUND)
  PCL_ADD_EXECUTABLE(pcl_converter ${SUBSYS_NAME} converter.cpp)
endif()
PCL_ADD_EXECUTABLE(pcl_hdl_grabber ${SUBSYS_NAME} hdl_grabber_example.cpp)

#libply inherited tools
add_subdirectory(ply)
