PCL_ADD_TEST (outofcore_test test_outofcore 
              FILES test_outofcore.cpp
              LINK_WITH pcl_gtest pcl_common pcl_io pcl_filters pcl_outofcore
	      #ARGUMENTS "${PCL_SOURCE_DIR}/test/bun0.pcd"
	      )
