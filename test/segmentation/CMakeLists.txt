# Random walker requires Eigen::Sparse module that is available since 3.1.0
if(NOT ("${EIGEN_VERSION}" VERSION_LESS 3.1.0))
  PCL_ADD_TEST(random_walker test_random_walker
               FILES test_random_walker.cpp
               LINK_WITH pcl_gtest
               ARGUMENTS "${PCL_SOURCE_DIR}/test/segmentation/data")
endif()
