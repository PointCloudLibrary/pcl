# ---[ Unit tests
rosbuild_add_gtest (test_narf test_narf.cpp)
target_link_libraries (test_narf pcl_features)

rosbuild_add_gtest (test_vector_average test_vector_average.cpp)

rosbuild_add_gtest (test_kdtree test_kdtree.cpp)
target_link_libraries (test_kdtree pcl_kdtree)
rosbuild_add_gtest (test_common test_common.cpp)

rosbuild_add_gtest (test_io test_io.cpp)
target_link_libraries (test_io pcl_io)

rosbuild_add_gtest (test_feature test_feature.cpp)
rosbuild_add_openmp_flags (test_feature)
#target_link_libraries (test_feature pcl_io tbb pcl_kdtree pcl_features)
target_link_libraries (test_feature pcl_io pcl_kdtree pcl_features)

rosbuild_add_gtest (test_sample_consensus test_sample_consensus.cpp)
target_link_libraries (test_sample_consensus pcl_io pcl_sample_consensus pcl_kdtree pcl_features)

rosbuild_add_gtest (test_filters test_filters.cpp)
target_link_libraries (test_filters pcl_filters pcl_io pcl_sample_consensus pcl_kdtree)

rosbuild_add_gtest (test_transforms test_transforms.cpp)
target_link_libraries (test_transforms pcl_io)

rosbuild_add_gtest (test_segmentation test_segmentation.cpp)
target_link_libraries (test_segmentation pcl_io pcl_segmentation)

rosbuild_add_gtest (test_iterators test_iterators.cpp)
target_link_libraries (test_iterators pcl_io)

rosbuild_add_gtest (test_registration test_registration.cpp)
target_link_libraries (test_registration pcl_io pcl_registration pcl_features)
rosbuild_link_boost (test_registration system filesystem)

rosbuild_add_gtest (test_keypoints test_keypoints.cpp)
target_link_libraries (test_keypoints pcl_io pcl_kdtree pcl_filters)

rosbuild_add_gtest (test_surface test_surface.cpp)
target_link_libraries (test_surface pcl_io pcl_kdtree pcl_surface pcl_features)

rosbuild_add_gtest (test_ii_normals test_ii_normals.cpp)
target_link_libraries (test_ii_normals pcl_io pcl_features)

#rosbuild_add_gtest (test_mls test_mls.cpp)
rosbuild_add_executable (test_mls test_mls.cpp)
target_link_libraries (test_mls pcl_io pcl_kdtree pcl_surface pcl_filters)
