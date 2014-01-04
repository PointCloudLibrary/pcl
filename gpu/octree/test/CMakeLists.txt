if(BUILD_TESTS)
	set(the_test_target test_gpu_octree)

	get_filename_component(DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
	get_filename_component(INC "${DIR}/../../../octree/include" REALPATH)
  include_directories("${INC}")

  FILE(GLOB test_src *.cpp *.hpp)
  #PCL_ADD_TEST(a_gpu_octree_test "${the_test_target}" FILES ${test_src} LINK_WITH pcl_octree pcl_gpu_containers pcl_gpu_octree)
  #add_dependencies("${the_test_target}" pcl_octree pcl_gpu_containes pcl_gpu_octree)
	
endif(BUILD_TESTS)
