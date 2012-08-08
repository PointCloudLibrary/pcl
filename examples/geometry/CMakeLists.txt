if(BUILD_geometry AND BUILD_pcl_half_edge_mesh)

  PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS geometry)

  PCL_ADD_EXAMPLE(pcl_example_half_edge_mesh FILES example_half_edge_mesh.cpp LINK_WITH pcl_geometry)

endif()
