if(BUILD_geometry)

  # BUILD_pcl_half_edge_mesh is defined in another scope
  get_directory_property(BUILD_pcl_half_edge_mesh DIRECTORY "../../geometry" DEFINITION "BUILD_pcl_half_edge_mesh")

  if(BUILD_pcl_half_edge_mesh)
    PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS geometry)
    PCL_ADD_EXAMPLE(pcl_example_half_edge_mesh FILES example_half_edge_mesh.cpp LINK_WITH pcl_geometry)
  endif()

endif()



