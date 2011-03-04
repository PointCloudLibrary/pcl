# ---[ Point Cloud Library - Segmentation
rosbuild_add_library (pcl_segmentation
                      segment_differences.cpp
                      sac_segmentation.cpp
                      extract_clusters.cpp
                      extract_polygonal_prism_data.cpp
                     )
rosbuild_add_compile_flags (pcl_segmentation ${SSE_FLAGS})
target_link_libraries (pcl_segmentation pcl_sample_consensus pcl_kdtree)
rosbuild_link_boost (pcl_segmentation system filesystem)

