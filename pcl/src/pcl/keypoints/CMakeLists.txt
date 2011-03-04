# ---[ Point Cloud Library - Keypoints
rosbuild_add_library (pcl_keypoints
                      narf_keypoint.cpp
                     )
target_link_libraries (pcl_keypoints pcl_range_image_border_extractor)
rosbuild_add_compile_flags (pcl_keypoints ${SSE_FLAGS})
rosbuild_add_openmp_flags (pcl_keypoints)

