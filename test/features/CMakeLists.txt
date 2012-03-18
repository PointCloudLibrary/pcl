PCL_ADD_TEST(feature_base test_base_feature
             FILES test_base_feature.cpp
             LINK_WITH pcl_features pcl_io
             ARGUMENTS ${PCL_SOURCE_DIR}/test/bun0.pcd)
PCL_ADD_TEST(feature_normal_estimation test_normal_estimation
             FILES test_normal_estimation.cpp
             LINK_WITH pcl_features pcl_io
             ARGUMENTS ${PCL_SOURCE_DIR}/test/bun0.pcd)
PCL_ADD_TEST(feature_pfh_estimation test_pfh_estimation
             FILES test_pfh_estimation.cpp
             LINK_WITH pcl_features pcl_io
             ARGUMENTS ${PCL_SOURCE_DIR}/test/bun0.pcd)

PCL_ADD_TEST(feature_cvfh_estimation test_cvfh_estimation
             FILES test_cvfh_estimation.cpp
             LINK_WITH pcl_features pcl_io pcl_filters
             ARGUMENTS ${PCL_SOURCE_DIR}/test/bun0.pcd ${PCL_SOURCE_DIR}/test/milk.pcd)

PCL_ADD_TEST(feature_ppf_estimation test_ppf_estimation
             FILES test_ppf_estimation.cpp
             LINK_WITH pcl_features pcl_io
             ARGUMENTS ${PCL_SOURCE_DIR}/test/bun0.pcd)
PCL_ADD_TEST(feature_shot_estimation test_shot_estimation
             FILES test_shot_estimation.cpp
             LINK_WITH pcl_features pcl_io
             ARGUMENTS ${PCL_SOURCE_DIR}/test/bun0.pcd)
PCL_ADD_TEST(feature_boundary_estimation test_boundary_estimation
             FILES test_boundary_estimation.cpp
             LINK_WITH pcl_features pcl_io
             ARGUMENTS ${PCL_SOURCE_DIR}/test/bun0.pcd)
PCL_ADD_TEST(feature_curvatures_estimation test_curvatures_estimation
             FILES test_curvatures_estimation.cpp
             LINK_WITH pcl_features pcl_io
             ARGUMENTS ${PCL_SOURCE_DIR}/test/bun0.pcd)
PCL_ADD_TEST(feature_spin_estimation test_spin_estimation
             FILES test_spin_estimation.cpp
             LINK_WITH pcl_features pcl_io
             ARGUMENTS ${PCL_SOURCE_DIR}/test/bun0.pcd)
PCL_ADD_TEST(feature_invariants_estimation test_invariants_estimation
             FILES test_invariants_estimation.cpp
             LINK_WITH pcl_features pcl_io
             ARGUMENTS ${PCL_SOURCE_DIR}/test/bun0.pcd)
PCL_ADD_TEST(feature_gradient_estimation test_gradient_estimation
             FILES test_gradient_estimation.cpp
             LINK_WITH pcl_features)
PCL_ADD_TEST(feature_rift_estimation test_rift_estimation
             FILES test_rift_estimation.cpp
             LINK_WITH pcl_features)
PCL_ADD_TEST(feature_board_estimation test_board_estimation
             FILES test_board_estimation.cpp
             LINK_WITH pcl_features pcl_io
             ARGUMENTS ${PCL_SOURCE_DIR}/test/bun0.pcd)
PCL_ADD_TEST(feature_shot_lrf_estimation test_shot_lrf_estimation
             FILES test_shot_lrf_estimation.cpp
             LINK_WITH pcl_features pcl_io
             ARGUMENTS ${PCL_SOURCE_DIR}/test/bun0.pcd)
