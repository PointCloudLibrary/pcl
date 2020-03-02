
if(NOT (GLEW_FOUND AND GLUT_FOUND))
  return()
endif()

include_directories(SYSTEM ${VTK_INCLUDE_DIRS})

set(srcs simulation_io.cpp)
set(incs simulation_io.hpp)
set(LIB_NAME pcl_simulation_io)

PCL_ADD_LIBRARY(${LIB_NAME} COMPONENT ${SUBSYS_NAME} SOURCES
  ${srcs} ${incs} ${compression_incs} ${impl_incs}
  ${VTK_IO_TARGET_LINK_LIBRARIES}
  ${OPENNI_INCLUDES})
target_link_libraries(${LIB_NAME} pcl_simulation pcl_common pcl_io
  ${VTK_IO_TARGET_LINK_LIBRARIES} ${OPENGL_LIBRARIES} GLUT::GLUT)
