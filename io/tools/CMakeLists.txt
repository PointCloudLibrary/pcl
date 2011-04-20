if(OPENNI_FOUND)
  PCL_ADD_EXECUTABLE(openni_grabber_example ${SUBSYS_NAME} openni_grabber_example.cpp)
  target_link_libraries(openni_grabber_example pcl_common pcl_io ${Boost_LIBRARIES})
endif(OPENNI_FOUND)

