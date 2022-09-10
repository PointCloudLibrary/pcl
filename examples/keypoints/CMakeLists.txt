if(NOT BUILD_visualization)
  return()
endif()

PCL_SUBSYS_DEPEND(build NAME ${SUBSYS_NAME} DEPS visualization)

## Find VTK
if(NOT VTK_FOUND)
  set(DEFAULT FALSE)
  set(REASON "VTK was not found.")
else()
  set(DEFAULT TRUE)
  set(REASON)
endif()

PCL_ADD_EXAMPLE(pcl_example_sift_keypoint_estimation FILES example_sift_keypoint_estimation.cpp
                LINK_WITH pcl_common  pcl_visualization pcl_keypoints pcl_io)
PCL_ADD_EXAMPLE(pcl_example_sift_normal_keypoint_estimation FILES example_sift_normal_keypoint_estimation.cpp
                LINK_WITH pcl_common  pcl_visualization pcl_keypoints pcl_io)
PCL_ADD_EXAMPLE(pcl_example_sift_z_keypoint_estimation FILES example_sift_z_keypoint_estimation.cpp
                LINK_WITH pcl_common  pcl_visualization pcl_keypoints pcl_io)
PCL_ADD_EXAMPLE(pcl_example_get_keypoints_indices FILES example_get_keypoints_indices.cpp
                LINK_WITH pcl_common pcl_keypoints pcl_io)
