set(SUBSYS_NAME geometry)
set(SUBSYS_DESC "Point cloud geometry library")
set(SUBSYS_DEPS common)

set(build TRUE)
PCL_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON)
PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS})

PCL_ADD_DOC(${SUBSYS_NAME})

if(build)

  set(incs
      include/pcl/${SUBSYS_NAME}/eigen.h
      include/pcl/${SUBSYS_NAME}/planar_polygon.h
      include/pcl/${SUBSYS_NAME}/organized_index_iterator.h
      include/pcl/${SUBSYS_NAME}/line_iterator.h
      include/pcl/${SUBSYS_NAME}/polygon_operations.h
     )

  set(impl_incs
      include/pcl/${SUBSYS_NAME}/impl/polygon_operations.hpp
     )

  set(srcs
      src/geometry.cpp
     )

  set(half_edge_mesh_incs
      include/pcl/${SUBSYS_NAME}/mesh_element_index.h
     )

  set(half_edge_mesh_impl_incs
      include/pcl/${SUBSYS_NAME}/impl/face.hpp
      include/pcl/${SUBSYS_NAME}/impl/half_edge.hpp
      include/pcl/${SUBSYS_NAME}/impl/mesh_base.hpp
      include/pcl/${SUBSYS_NAME}/impl/mesh_circulators.hpp
      include/pcl/${SUBSYS_NAME}/impl/mesh_topology.hpp
      include/pcl/${SUBSYS_NAME}/impl/triangle_mesh.hpp
      include/pcl/${SUBSYS_NAME}/impl/vertex.hpp
     )

  set(half_edge_mesh_srcs
      src/mesh_element_index.cpp
     )

  # NOTE: mesh_circulators.hpp currently uses boost::conditional which only exists for versions > 1.44
  if(Boost_MAJOR_VERSION GREATER 1 OR Boost_MINOR_VERSION GREATER 44)
    set(BUILD_pcl_half_edge_mesh TRUE)
    set(incs ${incs} ${half_edge_mesh_incs})
    set(srcs ${srcs} ${half_edge_mesh_srcs})
    set(impl_incs ${impl_incs} ${half_edge_mesh_impl_incs})
  else()
    set(BUILD_pcl_half_edge_mesh FALSE)
  endif()

  set(LIB_NAME pcl_${SUBSYS_NAME})
  include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
  PCL_ADD_LIBRARY(${LIB_NAME} ${SUBSYS_NAME} ${srcs} ${incs} ${impl_incs})
  target_link_libraries(${LIB_NAME} pcl_common)
  PCL_MAKE_PKGCONFIG(${LIB_NAME} ${SUBSYS_NAME} "${SUBSYS_DESC}" "${SUBSYS_DEPS}" "" "" "" "")
  # Install include files
  PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME} ${incs})
  PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME}/impl ${impl_incs})

endif(build)
