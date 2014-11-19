find_package(GLEW)
if(GLEW_FOUND)
  find_package(GLUT)
  if(GLUT_FOUND)
    include_directories(${GLUT_INCLUDE_DIR} ${GLEW_INCLUDE_DIR})
    include_directories(${VTK_INCLUDE_DIRS})

    PCL_ADD_EXECUTABLE(pcl_sim_viewer ${SUBSYS_NAME} sim_viewer.cpp)
    target_link_libraries (pcl_sim_viewer   
      ${VTK_IO_TARGET_LINK_LIBRARIES}    pcl_kdtree 
      pcl_simulation   pcl_common  pcl_io pcl_visualization   
      ${GLEW_LIBRARIES} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES})

    PCL_ADD_EXECUTABLE(pcl_sim_test_simple ${SUBSYS_NAME} sim_test_simple.cpp)
    target_link_libraries (pcl_sim_test_simple
      ${VTK_IO_TARGET_LINK_LIBRARIES}   
      pcl_simulation   pcl_common  pcl_io pcl_visualization 
      ${GLEW_LIBRARIES} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES})

    PCL_ADD_EXECUTABLE(pcl_sim_test_performance ${SUBSYS_NAME} sim_test_performance.cpp)
    target_link_libraries (pcl_sim_test_performance 
      ${VTK_IO_TARGET_LINK_LIBRARIES} 
      pcl_simulation   pcl_common  pcl_io pcl_visualization
      ${GLEW_LIBRARIES} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES})

    set(srcs simulation_io.cpp )
    set(incs simulation_io.hpp )
    set(LIB_NAME pcl_simulation_io)
    PCL_ADD_LIBRARY(${LIB_NAME} ${SUBSYS_NAME}
      ${srcs} ${incs} ${compression_incs} ${impl_incs}
      ${VTK_IO_TARGET_LINK_LIBRARIES} 
      ${OPENNI_INCLUDES})
    target_link_libraries(${LIB_NAME} pcl_simulation pcl_common pcl_io 
	    ${VTK_IO_TARGET_LINK_LIBRARIES} ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES})

    PCL_ADD_EXECUTABLE(pcl_sim_terminal_demo ${SUBSYS_NAME} sim_terminal_demo.cpp)
    target_link_libraries (pcl_sim_terminal_demo
	    ${VTK_IO_TARGET_LINK_LIBRARIES}    
	    pcl_simulation   pcl_common  pcl_io pcl_visualization pcl_simulation_io
	    ${GLEW_LIBRARIES} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES})
  endif(GLUT_FOUND)
endif (GLEW_FOUND)
