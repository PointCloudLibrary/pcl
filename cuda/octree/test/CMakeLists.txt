if(BUILD_TESTS)
    find_package(OpenCV QUIET)
	
	set(the_test_target test_gpu_octree)

	get_filename_component(DIR ${CMAKE_CURRENT_LIST_FILE} PATH)
	get_filename_component(INC ${DIR}/../../../octree/include REALPATH)
    include_directories(${INC} ${OpenCV_INCLUDE_DIRS})

    FILE(GLOB test_src *.cpp *.hpp)
    PCL_ADD_TEST(a_gpu_octree_test ${the_test_target} FILES ${test_src} LINK_WITH pcl_octree pcl_gpu_containers pcl_gpu_octree)    
	add_dependencies(${the_test_target} pcl_octree pcl_gpu_containes pcl_gpu_octree)
	
	if (OpenCV_FOUND)
		include_directories(${OpenCV_INCLUDE_DIRS})		
		target_link_libraries(${the_test_target} ${OpenCV_LIBS})		
		add_definitions(-DHAVE_OPENCV)	
    endif()
endif(BUILD_TESTS)
