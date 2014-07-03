if(BUILD_simulation)

  PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS simulation)
  PCL_ADD_EXAMPLE(pcl_example_shape_generator FILES example_shape_generator.cpp LINK_WITH pcl_common pcl_simulation pcl_io)
  
endif(BUILD_simulation)
