PCL_ADD_TEST (a_outofcore_test test_outofcore
              FILES test_outofcore.cpp
              LINK_WITH pcl_outofcore)
link_ros_libs (test_outofcore)
