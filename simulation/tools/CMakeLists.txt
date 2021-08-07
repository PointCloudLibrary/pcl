set(SUBSYS_NAME tools)

find_package(GLUT)

if(NOT (GLEW_FOUND AND GLUT_FOUND))
  return()
endif()

if(APPLE)
  # GLUT::GLUT is not properly set on macos
  set(_glut_target ${GLUT_glut_LIBRARY})
else()
  set(_glut_target GLUT::GLUT)
endif()

include_directories(SYSTEM ${VTK_INCLUDE_DIRS})

PCL_ADD_EXECUTABLE(pcl_sim_viewer COMPONENT ${SUBSYS_NAME} SOURCES sim_viewer.cpp)
target_link_libraries (pcl_sim_viewer
  ${VTK_IO_TARGET_LINK_LIBRARIES}    pcl_kdtree
  pcl_simulation   pcl_common  pcl_io pcl_visualization
  GLEW::GLEW ${_glut_target} ${OPENGL_LIBRARIES})

PCL_ADD_EXECUTABLE(pcl_sim_test_simple COMPONENT ${SUBSYS_NAME} SOURCES sim_test_simple.cpp)
target_link_libraries (pcl_sim_test_simple
  ${VTK_IO_TARGET_LINK_LIBRARIES}
  pcl_simulation   pcl_common  pcl_io pcl_visualization
  GLEW::GLEW ${_glut_target} ${OPENGL_LIBRARIES})

PCL_ADD_EXECUTABLE(pcl_sim_test_performance COMPONENT ${SUBSYS_NAME} SOURCES sim_test_performance.cpp)
target_link_libraries (pcl_sim_test_performance
  ${VTK_IO_TARGET_LINK_LIBRARIES}
  pcl_simulation   pcl_common  pcl_io pcl_visualization
  GLEW::GLEW ${_glut_target} ${OPENGL_LIBRARIES})

set(srcs simulation_io.cpp)
set(incs simulation_io.hpp)
set(LIB_NAME pcl_simulation_io)
PCL_ADD_LIBRARY(${LIB_NAME} COMPONENT ${SUBSYS_NAME} SOURCES
  ${srcs} ${incs} ${compression_incs} ${impl_incs}
  ${VTK_IO_TARGET_LINK_LIBRARIES}
  ${OPENNI_INCLUDES})
target_link_libraries(${LIB_NAME} pcl_simulation pcl_common pcl_io
  ${VTK_IO_TARGET_LINK_LIBRARIES} ${OPENGL_LIBRARIES} ${_glut_target} GLEW::GLEW)

PCL_ADD_EXECUTABLE(pcl_sim_terminal_demo COMPONENT ${SUBSYS_NAME} SOURCES sim_terminal_demo.cpp)
target_link_libraries (pcl_sim_terminal_demo
  ${VTK_IO_TARGET_LINK_LIBRARIES}
  pcl_simulation   pcl_common  pcl_io pcl_visualization pcl_simulation_io
  GLEW::GLEW ${_glut_target} ${OPENGL_LIBRARIES})
