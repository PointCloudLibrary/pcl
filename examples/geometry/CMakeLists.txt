if(NOT BUILD_geometry)
  return()
endif()

PCL_SUBSYS_DEPEND(build NAME ${SUBSYS_NAME} DEPS geometry)
PCL_ADD_EXAMPLE(pcl_example_half_edge_mesh FILES example_half_edge_mesh.cpp LINK_WITH pcl_common)
