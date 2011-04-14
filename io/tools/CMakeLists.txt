if(OPENNI_FOUND)
  PCL_ADD_EXECUTABLE(kinect_grabber_example ${SUBSYS_NAME} kinect_grabber_example.cpp)
  target_link_libraries(kinect_grabber_example pcl_common pcl_io ${Boost_LIBRARIES})
endif(OPENNI_FOUND)

