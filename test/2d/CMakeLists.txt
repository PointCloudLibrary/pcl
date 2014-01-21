PCL_ADD_TEST(test_2d test_2d FILES test_2d.cpp 
             LINK_WITH pcl_io pcl_gtest
             ARGUMENTS "${PCL_SOURCE_DIR}/test/2d/lena.pcd"
                       "${PCL_SOURCE_DIR}/test/2d/gauss_smooth.pcd"
                       "${PCL_SOURCE_DIR}/test/2d/erosion.pcd"
                       "${PCL_SOURCE_DIR}/test/2d/dilation.pcd"
                       "${PCL_SOURCE_DIR}/test/2d/opening.pcd"
                       "${PCL_SOURCE_DIR}/test/2d/closing.pcd"
                       "${PCL_SOURCE_DIR}/test/2d/erosion_binary.pcd"
                       "${PCL_SOURCE_DIR}/test/2d/dilation_binary.pcd"
                       "${PCL_SOURCE_DIR}/test/2d/opening_binary.pcd"
                       "${PCL_SOURCE_DIR}/test/2d/closing_binary.pcd"
                       "${PCL_SOURCE_DIR}/test/2d/canny.pcd")
