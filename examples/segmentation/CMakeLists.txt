if(BUILD_visualization)

  PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS visualization)

  PCL_ADD_EXAMPLE(pcl_example_extract_clusters_normals FILES example_extract_clusters_normals.cpp
		                LINK_WITH pcl_common pcl_keypoints pcl_io pcl_segmentation)
  PCL_ADD_EXAMPLE(pcl_example_region_growing FILES example_region_growing.cpp
		                LINK_WITH pcl_common pcl_keypoints pcl_io pcl_segmentation)
endif(BUILD_visualization)
