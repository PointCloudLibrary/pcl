set(SUBSYS_NAME simulation_io)
set(SUBSYS_DESC "Point Cloud Library Simulation I/O")
set(SUBSYS_DEPS common io surface kdtree features search octree visualization filters geometry)

set(DEFAULT TRUE)
#Requires glew
if(NOT GLEW_FOUND)
  set(DEFAULT FALSE)
  set(REASON "GLEW was not found.")
endif()

if(NOT GLUT_FOUND)
  set(DEFAULT FALSE)
  set(REASON "GLUT was not found.")
endif()

PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ${DEFAULT} "${REASON}")
PCL_SUBSYS_DEPEND(build "${SUBSYS_NAME}" DEPS ${SUBSYS_DEPS} EXT_DEPS opengl glew glut)

if(NOT build)
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
