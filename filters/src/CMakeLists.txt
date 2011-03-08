# ---[ Point Cloud Library - Filters
rosbuild_add_library (pcl_filters
                      filter.cpp
                      conditional_removal.cpp
                      passthrough.cpp
                      voxel_grid.cpp
                      extract_indices.cpp
                      project_inliers.cpp
                      radius_outlier_removal.cpp
                      statistical_outlier_removal.cpp
                     )
rosbuild_add_compile_flags (pcl_filters ${SSE_FLAGS})
rosbuild_link_boost (pcl_filters system filesystem)
target_link_libraries (pcl_filters pcl_base pcl_sample_consensus pcl_kdtree)

