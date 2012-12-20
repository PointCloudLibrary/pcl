if(BUILD_visualization)

  PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS visualization segmentation)

  PCL_ADD_EXAMPLE(pcl_example_extract_clusters_normals FILES example_extract_clusters_normals.cpp
		                LINK_WITH pcl_common pcl_keypoints pcl_io pcl_segmentation)
  PCL_ADD_EXAMPLE(pcl_example_region_growing FILES example_region_growing.cpp
		                LINK_WITH pcl_common pcl_keypoints pcl_io pcl_segmentation)

  ## Find VTK 
  if(VTK_FOUND)
    include (${VTK_USE_FILE})
    PCL_ADD_EXAMPLE(pcl_example_supervoxels FILES example_supervoxels.cpp
                    LINK_WITH pcl_common pcl_features pcl_segmentation pcl_octree pcl_kdtree pcl_visualization)
  endif(VTK_FOUND)
endif(BUILD_visualization)
