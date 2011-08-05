set (SUBSYS_NAME tools)
set (SUBSYS_DESC "Useful PCL-based command line tools")
set (SUBSYS_DEPS common io filters sample_consensus segmentation kdtree features surface octree registration)

PCL_SUBSYS_OPTION (build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} ${REASON})
PCL_SUBSYS_DEPEND (build ${SUBSYS_NAME} ${SUBSYS_DEPS})

if (build)

  PCL_ADD_EXECUTABLE (normal_estimation ${SUBSYS_NAME} normal_estimation.cpp)
  target_link_libraries (normal_estimation pcl_common pcl_io pcl_features pcl_kdtree)

  PCL_ADD_EXECUTABLE (voxel_grid ${SUBSYS_NAME} voxel_grid.cpp)
  target_link_libraries (voxel_grid pcl_common pcl_io pcl_filters)
	
	PCL_ADD_EXECUTABLE (passthrough_filter ${SUBSYS_NAME} passthrough_filter.cpp)
	target_link_libraries (passthrough_filter pcl_common pcl_io pcl_filters)

endif ()
