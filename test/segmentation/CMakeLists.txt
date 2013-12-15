PCL_ADD_TEST(random_walker test_random_walker
             FILES test_random_walker.cpp
             LINK_WITH pcl_gtest
             ARGUMENTS ${PCL_SOURCE_DIR}/test/segmentation/data)
