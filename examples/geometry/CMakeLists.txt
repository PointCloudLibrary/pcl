if(BUILD_geometry)

  PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS geometry)
  PCL_ADD_EXAMPLE(pcl_example_half_edge_mesh FILES example_half_edge_mesh.cpp LINK_WITH pcl_common)

endif()



