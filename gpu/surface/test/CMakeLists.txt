if(NOT BUILD_TESTS)
  return()
endif()

set(the_test_target test_gpu_surface)

file(GLOB test_src *.cpp *.hpp)
#PCL_ADD_TEST(a_gpu_surface_test ${the_test_target} FILES ${test_src} LINK_WITH pcl_io pcl_gpu_containers pcl_gpu_surface pcl_visualization pcl_surface pcl_octree pcl_kdtree pcl_search)
#add_dependencies(${the_test_target} pcl_io pcl_gpu_containes pcl_gpu_surface pcl_visualization)
