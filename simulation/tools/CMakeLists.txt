find_package(GLEW)

if(GLEW_FOUND)
  PCL_ADD_EXECUTABLE(range_viewer ${SUBSYS_NAME} range_viewer.cpp)
  target_link_libraries(range_viewer pcl_common pcl_io pcl_visualization ${GLEW_LIBRARIES})

  PCL_ADD_EXECUTABLE(range_viewer_v2 ${SUBSYS_NAME} range_viewer_v2.cpp)
  target_link_libraries(range_viewer_v2 pcl_common pcl_io pcl_visualization pcl_simulation ${GLEW_LIBRARIES})

  find_package(GLUT)
  if(GLUT_FOUND)
    include_directories(${GLUT_INCLUDE_DIR} ${GLEW_INCLUDE_DIR})
  
    #PCL_ADD_EXECUTABLE(range_test ${SUBSYS_NAME} range_test.cpp)
    #target_link_libraries(range_test pcl_common pcl_simulation pcl_io ${GLEW_LIBRARIES} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES})

    include_directories(${VTK_INCLUDE_DIRS})
    PCL_ADD_EXECUTABLE(range_test_v2 ${SUBSYS_NAME} range_test_v2.cpp)
    target_link_libraries (range_test_v2 
      ${VTK_IO_TARGET_LINK_LIBRARIES} 
      pcl_simulation   pcl_common  pcl_io pcl_visualization
      ${GLEW_LIBRARIES} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES})

    PCL_ADD_EXECUTABLE(range_performance_test ${SUBSYS_NAME} range_performance_test.cpp)
    target_link_libraries (range_performance_test 
      ${VTK_IO_TARGET_LINK_LIBRARIES} 
      pcl_simulation   pcl_common  pcl_io pcl_visualization
      ${GLEW_LIBRARIES} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES})

#  PCL_ADD_EXECUTABLE(test_viewer ${SUBSYS_NAME} test_viewer.cpp)
#  target_link_libraries (test_viewer
#    ${VTK_IO_TARGET_LINK_LIBRARIES} 
#    pcl_simulation   pcl_common  pcl_io pcl_visualization 
#    ${GLEW_LIBRARIES} ${OPENGL_LIBRARIES})
                 
  endif(GLUT_FOUND)
endif (GLEW_FOUND)