
if(BUILD_visualization)

  PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS visualization)

  PCL_ADD_EXAMPLE(pcl_example_stereo_baseline FILES example_stereo_baseline.cpp
		                LINK_WITH pcl_common  pcl_visualization pcl_stereo pcl_io)
						
endif(BUILD_visualization)