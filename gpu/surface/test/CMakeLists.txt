if(BUILD_TESTS)
	
	set(the_test_target test_gpu_surface)
	
  get_filename_component(DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
	get_filename_component(INC_SURF "${DIR}/../../surface/include" REALPATH)
	get_filename_component(INC_IO "${DIR}/../../../io/include" REALPATH)
	get_filename_component(INC_VIZ "${DIR}/../../../visualization/include" REALPATH)
	get_filename_component(INC_GEO "${DIR}/../../../geometry/include" REALPATH)
	get_filename_component(INC_SURF_CPU "${DIR}/../../../surface/include" REALPATH)
	get_filename_component(INC_SEA "${DIR}/../../../search/include" REALPATH)
	get_filename_component(INC_KD "${DIR}/../../../kdtree/include" REALPATH)
	get_filename_component(INC_OCT "${DIR}/../../../octree/include" REALPATH)
  include_directories("${INC_SURF}" "${INC_IO}" "${INC_VIZ}" "${INC_GEO}" "${INC_SURF_CPU}" "${INC_SEA}" "${INC_KD}" "${INC_OCT}" ${VTK_INCLUDE_DIRS})
	
  FILE(GLOB test_src *.cpp *.hpp)    
  #PCL_ADD_TEST(a_gpu_surface_test ${the_test_target} FILES ${test_src} LINK_WITH pcl_io pcl_gpu_containers pcl_gpu_surface pcl_visualization pcl_surface pcl_octree pcl_kdtree pcl_search)
  #add_dependencies(${the_test_target} pcl_io pcl_gpu_containes pcl_gpu_surface pcl_visualization)
			
endif(BUILD_TESTS)
