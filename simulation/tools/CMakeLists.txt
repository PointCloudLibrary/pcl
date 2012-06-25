
find_package(GLUT)
if(GLUT_FOUND)
  include_directories(${GLUT_INCLUDE_DIR} ${GLEW_INCLUDE_DIR})
  include_directories(${VTK_INCLUDE_DIRS})

  PCL_ADD_EXECUTABLE(pcl_sim_viewer ${SUBSYS_NAME} sim_viewer.cpp)
  target_link_libraries (pcl_sim_viewer   
      pcl_kdtree pcl_simulation pcl_common pcl_io pcl_visualization
      ${GLEW_LIBRARIES} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES})

  PCL_ADD_EXECUTABLE(pcl_sim_test_simple ${SUBSYS_NAME} sim_test_simple.cpp)
  target_link_libraries (pcl_sim_test_simple
      pcl_simulation pcl_common pcl_io pcl_visualization
      ${GLEW_LIBRARIES} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES})

  PCL_ADD_EXECUTABLE(pcl_sim_test_performance ${SUBSYS_NAME} sim_test_performance.cpp)
  target_link_libraries (pcl_sim_test_performance 
      pcl_simulation pcl_common pcl_io pcl_visualization
      ${GLEW_LIBRARIES} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES})

endif(GLUT_FOUND)

