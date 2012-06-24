PCL_ADD_TEST(test_2d 2d FILES test_2d.cpp LINK_WITH pcl_2d
             ARGUMENTS ${PCL_SOURCE_DIR}/test/2d/lena-grayscale.png
                       ${PCL_SOURCE_DIR}/test/2d/gauss_smooth.png)
