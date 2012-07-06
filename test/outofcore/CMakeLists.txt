PCL_ADD_TEST (outofcore_test test_outofcore 
              FILES test_outofcore.cpp
              LINK_WITH pcl_gtest pcl_common pcl_io pcl_filters pcl_outofcore)
