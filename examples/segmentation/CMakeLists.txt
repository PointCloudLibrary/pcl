if(NOT BUILD_visualization)
  return()
endif()

PCL_SUBSYS_DEPEND(build NAME ${SUBSYS_NAME} DEPS visualization segmentation)

PCL_ADD_EXAMPLE(pcl_example_extract_clusters_normals FILES example_extract_clusters_normals.cpp
                LINK_WITH pcl_common pcl_keypoints pcl_io pcl_segmentation)
PCL_ADD_EXAMPLE(pcl_example_region_growing FILES example_region_growing.cpp
                LINK_WITH pcl_common pcl_keypoints pcl_io pcl_segmentation)

## Find VTK
if(VTK_FOUND)
  PCL_ADD_EXAMPLE(pcl_example_supervoxels FILES example_supervoxels.cpp
                  LINK_WITH pcl_common pcl_features pcl_segmentation pcl_octree pcl_kdtree pcl_visualization)
  PCL_ADD_EXAMPLE(pcl_example_lccp_segmentation FILES example_lccp_segmentation.cpp
                  LINK_WITH pcl_common pcl_io pcl_segmentation pcl_visualization)
  PCL_ADD_EXAMPLE(pcl_example_cpc_segmentation FILES example_cpc_segmentation.cpp
                  LINK_WITH pcl_common pcl_io pcl_segmentation pcl_visualization pcl_sample_consensus)
endif()
