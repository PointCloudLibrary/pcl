if (BUILD_outofcore)

  PCL_SUBSYS_DEPEND (build ${SUBSYS_NAME} DEPS outofcore io common octree filters)

  PCL_ADD_EXAMPLE (pcl_example_outofcore_with_lod FILES pcl_example_outofcore_with_lod.cpp LINK_WITH pcl_outofcore pcl_common pcl_io pcl_octree pcl_filters)

  PCL_ADD_EXAMPLE (pcl_example_outofcore FILES pcl_example_outofcore.cpp LINK_WITH pcl_outofcore pcl_common pcl_io pcl_octree pcl_filters)

endif (BUILD_outofcore)
